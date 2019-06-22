#include "scheduler.h"
#include "graph.h"
#include "car.h"

int RoadSimulator::getForwardJamDegree() {
	int sum = 0;
	for (auto d: forward) {
		sum += d.size();
	}
	return sum;
}

int RoadSimulator::getBackwardJamDegree() {
	int sum = 0;
	for (auto d: backward) {
		sum += d.size();
	}
	return sum;
}

int RoadSimulator::getForwardPresetJamDegree(Scheduler* p) {
	int sum = 0;
	for (auto d: forward) {
		for (int idx : d) {
			if ((p->cars[idx]).prior){
				sum++;
			}
		}
	}
	return sum;
}

int RoadSimulator::getBackwardPresetJamDegree(Scheduler* p) {
	int sum = 0;
	for (auto d: backward) {
		for (int idx : d) {
			if ((p->cars[idx]).prior){
				sum++;
			}
		}
	}
	return sum;
}



Scheduler::Scheduler(ifstream& carStream, ifstream& roadStream, ifstream& crossStream, ifstream& presetAnswerStream):
	graph(roadStream, crossStream) {
	string line;
	goCarSize = upperBound*2/3;
	stride = 50;
	curTime = 0;
	presetWay = priorWay = 0;
	sorted = false;
	home = way = end = 0;
	a = b = 0;
	onlyPreset = false;
	
	while (getline(carStream, line)) {
		if (line.empty() or line[0] == '#')
			continue;
		vector<int> temp(7);
		sscanf(line.c_str(), "(%d, %d, %d, %d, %d, %d, %d)", &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5], &temp[6]);
		cars.emplace_back(Car(temp));
	}

	Car::initState(cars.size());

	auto lambda = [](const Car& c1, const Car& c2)->bool {
		if (c1.prior and not c2.prior)
			return true;
		if (not c1.prior and c2.prior)
			return false;
		if (c1.planTime < c2.planTime)
			return true;
		if (c1.planTime > c2.planTime)
			return false;
		if (c1.maxSpeed > c2.maxSpeed)
			return true;
		if (c1.maxSpeed < c2.maxSpeed)
			return false;
		return c1.id < c2.id;
	};
	sort(cars.begin(), cars.end(), lambda);
	for (int i = 0; i < (int)cars.size(); ++i) {
		carIdxes.emplace(cars[i].id, i);
		if (cars[i].prior)
			priorCarIdxs.emplace_back(i);
	}



	garageCarList = vector<int>(cars.size(), 0);
	for (int i = 0; i < (int)garageCarList.size(); i++) {
		garageCarList[i] = i;
	}
	garageSize = (int)garageCarList.size();

	while (getline(presetAnswerStream, line)) {
		if (line.empty() or line[0] == '#')
			continue;
		for (char &ch : line) {
			if (ch == '(' or ch == ')' or ch == ',')
				ch = ' ';
		}
		stringstream ss;
		int id, carIdx, startTime, data;

		ss << line;
		ss >> id >> startTime;
		carIdx = getCarIdx(id);
		cars[carIdx].startTime = startTime;
		while (ss >> data)
			cars[carIdx].route.emplace_back(data);
	}
	computeFactor();
}

int Scheduler::getCarIdx(int id) {
	auto it = carIdxes.find(id);
	assert(it != carIdxes.end());
	return it->second;
}

void Scheduler::outputAnswer(ofstream& answerStream) {
    answerStream << "#(carId,StartTime,RoadId...)" << endl;
	for (Car &car : cars) {
		if (car.preset and not car.reset)
			continue;
		answerStream << "(" << car.id << ", " << car.startTime;
		for (int r : car.route)
			answerStream << ", " << r;
		answerStream << ")" << endl;
	}
}

void Scheduler::updateNextRoadSet() {
	auto pair = graph.floyd();
	dist = pair.first;
	next = pair.second;
}

void Scheduler::updateRoadJam() {
	for (int i = 0; i < (int)network.size(); i++) {
		graph.roads[i].forJam = network[i].getForwardJamDegree();
		graph.roads[i].backJam = network[i].getBackwardJamDegree();
		graph.roads[i].forPresetJam = network[i].getForwardPresetJamDegree(this);
		graph.roads[i].backPresetJam = network[i].getBackwardPresetJamDegree(this);
	}
}

void Scheduler::updateRoads(int targetTime) {
	while (!taskfinished() && curTime < targetTime) {
		run();
		++curTime;
	}
	for (int i = 0; i < (int)network.size(); i++) {
		graph.roads[i].forJam = network[i].getForwardJamDegree();
		graph.roads[i].backJam = network[i].getBackwardJamDegree();
	}
}

int Scheduler::getRoadAfterNowRoadIdx(int carIdx) {
	if (Car::getCarLocation(carIdx) == HOME) {
		assert(not cars[carIdx].route.empty());
		return cars[carIdx].route[0];
	}
	if (Car::getNowRoadIdx(carIdx) == cars[carIdx].route.size() - 1)
		return DESTINATION;
	return cars[carIdx].route[Car::getNowRoadIdx(carIdx) + 1];
}

bool Scheduler::decide(int carIdx) {
	assert(Car::getNextRoad(carIdx) == NOT_DECIDED);
	assert(carIdx < (int)cars.size());

	/*
	 * 若为预置车辆，则搜索当前道路后一条道路。
	 */
	if (cars[carIdx].preset and not cars[carIdx].reset) {
		Car::getNextRoad(carIdx) = getRoadAfterNowRoadIdx(carIdx);
		return true;
	}

	/*
	 * 若为非预置优先车辆，
	 * 未做出决策则采用Dijkstra做出决策，
	 * 否则搜索当前道路后一条道路。
	 */
	if (cars[carIdx].prior) {
		if (cars[carIdx].route.empty())
			cars[carIdx].route = graph.dijkstraForPrior(cars[carIdx]);
		assert(not cars[carIdx].route.empty());
		Car::getNextRoad(carIdx) = getRoadAfterNowRoadIdx(carIdx);
		return true;
	}

	if (Car::getToCross(carIdx) == cars[carIdx].dest) {
		Car::getNextRoad(carIdx) = DESTINATION;
		return true;
	}
	/*
	 * 若车辆进入上次决策道路失败，
	 * 则仍返回上次决策道路。
	 */
	if (not cars[carIdx].route.empty() and
		Car::getNowRoad(carIdx) != cars[carIdx].route.back()) {
		Car::getNextRoad(carIdx) = cars[carIdx].route.back();
		return true;
	}

	if (Car::getCarLocation(carIdx) == HOME) {
		int curCrossIdx = graph.getCrossIdx(cars[carIdx].src);
		int nextCrossIdx = next[curCrossIdx][graph.getCrossIdx(cars[carIdx].dest)];
		int nextRoadIdx = graph.getCrossRoadIdx(graph.crosses[curCrossIdx].id, graph.crosses[nextCrossIdx].id);
		Car::getNextRoad(carIdx) = graph.roads[nextRoadIdx].id;
		cars[carIdx].route.emplace_back(graph.roads[nextRoadIdx].id);
		return true;
	}


	int	curCrossIdx = graph.getCrossIdx(Car::getToCross(carIdx));
	int nextCrossIdx = next[curCrossIdx][graph.getCrossIdx(cars[carIdx].dest)];
	int nextRoadIdx = -1;
	/*
	 * 若最优路径下个路口为掉头路，
	 * 在调度规则中不合法，
	 * 则取当前路口中合法路径中最优解
	 */
	if (nextCrossIdx == graph.getCrossIdx(Car::getFromCross(carIdx))) {
		int nextRoadId = graph.dijkstra(cars[carIdx], Car::getToCross(carIdx), cars[carIdx].dest, Car::getNowRoad(carIdx));
		nextRoadIdx = graph.getRoadIdx(nextRoadId);
	} else {
		nextRoadIdx = graph.getCrossRoadIdx(graph.crosses[curCrossIdx].id, graph.crosses[nextCrossIdx].id);
	}
	if (Car::getCarLocation(carIdx) == HOME and graph.isRoadCongested(graph.roads[nextRoadIdx].id, curCrossIdx) and not cars[carIdx].reset) {
		return false;
	}
	Car::getNextRoad(carIdx) = graph.roads[nextRoadIdx].id;
	assert(Car::getNextRoad(carIdx) != Car::getNowRoad(carIdx));
	cars[carIdx].route.emplace_back(graph.roads[nextRoadIdx].id);
	return true;
}

bool Scheduler::readyToGo(int carIdx) {
	if (onlyPreset)
		return false;
	if (priorWay > 100 and not cars[carIdx].prior) {
		return (int)canGoCar.size() < goCarSize * 2 / 3 - priorWay;
	}
	return (int)canGoCar.size() < goCarSize;
}


void Scheduler::computeFactor() {
	double factor1 = (double)cars.size()/priorCarIdxs.size();
	
	int maxSpeed = 0, minSpeed = 0x3f3f3f3f, priorMax = 0, priorMin = 0x3f3f3f3f;
	int early = 0x3f3f3f3f, late = 0, priorEarly = 0x3f3f3f3f, priorLate = 0;
	set<int> src, dest, priorSrc, priorDest;
	for (Car& car: cars) {
		maxSpeed = max(car.maxSpeed, maxSpeed);
		minSpeed = min(car.maxSpeed, minSpeed);
		early = min(car.planTime, early);
		late = max(car.planTime, late);
		src.emplace(car.src);
		dest.emplace(car.dest);
		if (car.prior) {
			priorMax = max(car.maxSpeed, priorMax);
			priorMin = min(car.maxSpeed, priorMin);
			priorEarly = min(car.planTime, priorEarly);
			priorLate = max(car.planTime, priorLate);
			priorSrc.emplace(car.src);
			priorDest.emplace(car.dest);
		}
	}
	double s1 = (double)maxSpeed/minSpeed;
	double s2 = (double)priorMax/priorMin;
	double factor2 = s1/s2;

	double t1 = (double)late/early;
	double t2 = (double)priorLate/priorEarly;
	double factor3 = t1/t2;

	double factor4 = (double)src.size()/priorSrc.size();
	double factor5 = (double)dest.size()/priorDest.size();

	a = 0.05*factor1 + 0.2375*factor2 + 0.2375*factor3 + 0.2375*factor4 + 0.2375*factor5;
	b = 0.8*factor1 + 0.05*factor2 + 0.05*factor3 + 0.05*factor4 + 0.05*factor5;
}