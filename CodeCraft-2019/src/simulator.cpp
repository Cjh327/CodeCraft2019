#include "scheduler.h"
#include "graph.h"
#include "car.h"

vector<enum State> Car::states;
vector<int> Car::nextRoads;
vector<CarState> Car::instantStates;

void Scheduler::initNetwork() {
	home = (int)cars.size();
	way = 0;
	end = 0;
	presetWay = 0;

	network.resize(graph.roads.size());
	for (int i = 0; i < (int)graph.roads.size(); ++i) {
		network[i].forward.clear();
		auto laneNumber = (size_t)graph.roads[i].laneNumber;
		network[i].forward.resize(laneNumber);
		if (graph.roads[i].duplex) {
			network[i].backward.clear();
			network[i].backward.resize(laneNumber);
		}
	}
}

void Scheduler::saveFieldInfo() {
	FieldInfo fieldInfo;
	for (Car &car : cars)
		fieldInfo.carRoutes.emplace_back(car.route);

	fieldInfo.infoInstantStates = Car::instantStates;
	fieldInfo.infoNetwork = network;
	fieldInfo.infoGarageCarList = garageCarList;
	fieldInfo.infoGarageSize = garageSize;
	fieldInfo.infoCanGoCar = canGoCar;
	fieldInfo.infoHome = home;
	fieldInfo.infoWay = way;
	fieldInfo.infoEnd = end;
	fieldInfo.infoWaiting = waiting;
	fieldInfo.infoCurTime = curTime;
	fieldInfo.infoPresetWay = presetWay;
	fieldInfo.infoPriorWay = priorWay;
	fieldInfo.infoSorted = sorted;
	fieldInfo.infoGoCarSize = goCarSize;
	fieldInfoList.emplace_back(fieldInfo);
}

void Scheduler::recoverFieldInfo(int k) {
	assert(not fieldInfoList.empty());
	int i = max(0, (int)fieldInfoList.size() - k);
	FieldInfo& fieldInfo = fieldInfoList[i];
	for (int i = 0; i < (int)cars.size(); i++)
		cars[i].route = fieldInfo.carRoutes[i];

	Car::instantStates.assign(fieldInfo.infoInstantStates.begin(), fieldInfo.infoInstantStates.end());
	network.assign(fieldInfo.infoNetwork.begin(), fieldInfo.infoNetwork.end());
	garageCarList.assign(fieldInfo.infoGarageCarList.begin(), fieldInfo.infoGarageCarList.end());
	garageSize = fieldInfo.infoGarageSize;
	canGoCar = fieldInfo.infoCanGoCar;
	home = fieldInfo.infoHome;
	way = fieldInfo.infoWay;
	end = fieldInfo.infoEnd;
	waiting = fieldInfo.infoWaiting;
	curTime = fieldInfo.infoCurTime;
	presetWay = fieldInfo.infoPresetWay;
	priorWay = fieldInfo.infoPriorWay;
	sorted = fieldInfo.infoSorted;
	goCarSize = fieldInfo.infoGoCarSize;
	while (fieldInfoList.back().infoCurTime > fieldInfo.infoCurTime)
		fieldInfoList.pop_back();
}

void Scheduler::changeTenPercent() {
	initNetwork();
	saveFieldInfo();
	onlyPreset = true;
	vector<int> presetCars;
	for (int i = 0; i < (int)cars.size(); ++i) {
		if (cars[i].preset)
			presetCars.emplace_back(i);
	}
	while (end < presetCars.size()) {
		if (not run())
			assert(false);
		++curTime;
		// 如果死锁，则说明只跑预制车导致死锁
	}
	auto lambda = [this](int idx1, int idx2)->bool {
		if (this->cars[idx1].prior and not this->cars[idx2].prior)
			return true;
		if (not this->cars[idx1].prior and this->cars[idx2].prior)
			return false;
		return this->cars[idx1].reachTime > this->cars[idx2].reachTime;
	};
	sort(presetCars.begin(), presetCars.end(), lambda);
	recoverFieldInfo(1);
	for (int i = 0; i < (int)presetCars.size()/10; ++i) {
		cars[presetCars[i]].route.clear();
		cars[presetCars[i]].reset = true;
	}
	onlyPreset = false;
	curTime = 0;
}

void Scheduler::simulate() {
	initNetwork();

	const int interval = 20;
	bool block = false;
	int lastBlockTime = INF;
	int step = 2;
	curTime = 0;

	while (not taskfinished()) {
		if (block) {
			recoverFieldInfo(step);
			goCarSize = max(min(upperBound / max(1, step-1), goCarSize - 1000), 2000);
			stride = 4;
		} else if (curTime % interval == 0) {
			saveFieldInfo();
			stride += 5;
		}
		updateRoadJam();
		updateNextRoadSet();

		if (not run()) {
			cout << "-------------DEAD BLOCK: t = " << curTime << "---------------" << endl;
			updatePenalty();
			Car::freshState(cars.size());
			if (lastBlockTime/interval == curTime/interval)
				++step;
			else
				step = 2;
			lastBlockTime = curTime;
			block = true;
			for (int i = 0; i < (int)graph.roads.size(); i++) {
				if (i % 20 == 0 and i != 0) {
					cout << graph.roads[i].penalty << endl;
				} else {
					cout << graph.roads[i].penalty << "\t";
				}
			}
			cout << endl;
		} else {
			block = false;
			goCarSize = min(upperBound, goCarSize + stride);
			if (curTime > lastBlockTime) {
				for (Road &road : graph.roads)
					road.penalty = max(road.penalty - 0.01, 0.0);
			}
		}
		++curTime;
		cout << goCarSize << " " << step << endl;
	}

	int sum = 0, priSum = 0, priReach = 0, priGo = 0x3f3f3f3f;
	for (Car &car : cars) {
		sum += (car.reachTime - car.planTime);
		if (car.prior) {
			priSum += (car.reachTime - car.planTime);
			priReach = max(priReach, car.reachTime);
			priGo = min(priGo, car.planTime);
		}
	}

	cout << "Prior Schedule: " << priReach - priGo << endl;
	cout << "Total Schedule: " << curTime << endl;
	cout << "Prior Time: " << priSum << endl;
	cout << "Total Time: " << sum << endl;
	cout << "a: " << a << " b: " << b << endl;
	cout << "Schedule: " << (int)(a*(priReach - priGo) + curTime) << endl;
	cout << "Total: " << (int)(b*priSum + sum) << endl;
}

/**
 * 	taskfinished表示宏观上所有车辆是否到达目的地，
 */ 
bool Scheduler::taskfinished() {
	return (home == 0 and way == 0);
}

/**
 * 模拟路网状况，
 * 找到需要做出决策的小车并返回。
 * @args:
 * 		time: 当前时间
 */
void Scheduler::outputInfo(int time) {
	ofstream stream;
	stream.open("info.txt", ios::app);
	stream << time << " " << home << " " << way << " " << end << endl;
	stream.close();
}


bool Scheduler::run() {
	waiting = 0;

	initWaitList();

	driveJustCurrentRoad();

	driveCarInitList(true);


	if (not driveCarInWaitState()) {
		return false;
	}

	driveCarInitList(false);

	display();

	Car::freshState(cars.size());

	return true;
}

void Scheduler::driveJustCurrentRoad() {
	for (int i = 0; i < (int)network.size(); ++i) {
		if (not graph.roads[i].duplex)
			assert(network[i].backward.empty());
		updateRoadCars(network[i].forward, i);
		if (graph.roads[i].duplex)
			updateRoadCars(network[i].backward, i);
	}
}

bool Scheduler::driveCarInWaitState() {
	int curWaiting = waiting, preWaiting;
	while (curWaiting > 0) {
		for (Cross& cross : graph.crosses)
			updateCrossCars(cross);
		preWaiting = waiting;
		if (curWaiting == preWaiting)
			return false;
		curWaiting = preWaiting;
	}
	return true;
}

void Scheduler::updateRoadCars(vector<deque<int>>& road, int roadIdx) {
	for (int i = 0; i < (int)road.size(); ++i)
		updateRoadCars(road, roadIdx, i);
}

void Scheduler::updateRoadCars(vector<deque<int>>& road, int roadIdx, int laneIdx) {
	assert(laneIdx < (int)road.size());
	deque<int>& lane = road[laneIdx];

	int limitSpeed = graph.roads[roadIdx].speedLimit;
	for (int i = 0; i < (int)lane.size(); ++i) {
		int carIdx = lane[i];
		if (Car::getState(carIdx) == STOP)
			continue;
		if (i == 0) {
			if (Car::getCarOffset(carIdx) + min(limitSpeed, cars[carIdx].maxSpeed) > graph.roads[roadIdx].length) {
				if (Car::getState(carIdx) == READY)
					++waiting;

				Car::getState(carIdx) = WAITING;
				if (not decide(carIdx)) {
					assert(false);
				}
			} else {
				if (Car::getState(carIdx) == WAITING)
					--waiting;
				Car::getState(carIdx) = STOP;
				Car::getCarOffset(carIdx) += min(limitSpeed, cars[carIdx].maxSpeed);
			}
		} else {
			int nextCarIdx = lane[i - 1];
			if (Car::getCarOffset(carIdx) + min(limitSpeed, cars[carIdx].maxSpeed) < Car::getCarOffset(nextCarIdx)) {
				if (Car::getState(carIdx) == WAITING)
					--waiting;

				Car::getState(carIdx) = STOP;
				Car::getCarOffset(carIdx) += min(limitSpeed, cars[carIdx].maxSpeed);
			} else {
				if (Car::getState(nextCarIdx) == WAITING) {
					if (Car::getState(carIdx) == READY)
						++waiting;

					Car::getState(carIdx) = WAITING;
				} else if (Car::getState(nextCarIdx) == STOP) {
					if (Car::getState(carIdx) == WAITING)
						--waiting;

					Car::getState(carIdx) = STOP;
					Car::getCarOffset(carIdx) = Car::getCarOffset(nextCarIdx) - 1;
				} else {
					assert(false);
				}
			}
		}
	}
}

/**
 * 调度某个路口的车辆，
 * 由于可能存在依赖关系，
 * 因此循环调度路口所连接各道路
 */ 
void Scheduler::updateCrossCars(Cross& cross) {
	struct Temp {
		int roadId;
		int idx;
	};
	assert(cross.roads.size() == 4);
	vector<Temp> temp(cross.roads.size());
	for (int i = 0; i < (int)temp.size(); ++i) {
		temp[i].roadId = cross.roads[i];
		temp[i].idx = i;
	}
	sort(temp.begin(), temp.end(), [](const Temp& t1, const Temp& t2)->bool { return t1.roadId < t2.roadId; });
	for (auto &iter : temp) {
		if (iter.roadId == -1)
			continue;
		int roadIdx = graph.getRoadIdx(iter.roadId);
		vector<deque<int>>* lanes = inOutLanes(roadIdx, cross.id, true);
		if (lanes == nullptr)
			continue;
		
		int carIdx = -1;
		while (getCarFromSequeue(*lanes, carIdx)) {
			if (conflict(carIdx, iter.idx, cross))
				break;
			int oldLaneIdx = Car::getCarLaneIdx(carIdx);
			if (moveToNextRoad(carIdx, roadIdx, cross.id, (*lanes)[oldLaneIdx])) {
				updateRoadCars(*lanes, roadIdx, oldLaneIdx);
				assert(lanes == &network[roadIdx].forward or lanes == &network[roadIdx].backward);
				runCarInInitList(roadIdx, true, lanes == &network[roadIdx].forward);
			} else {
				break;
			}
		}
	}
}

bool Scheduler::getCarFromSequeue(const vector<deque<int>>& lanes, int& carIdx) {
	int offset = -1;
	bool prior = false;
	for (auto lane : lanes) {
		if (lane.empty() or Car::getState(lane[0]) != WAITING)
			continue;
		if (prior) {
			// 当前检索到的第一优先级为优先车
			if (not cars[lane[0]].prior)
				continue;
			if (Car::getCarOffset(lane[0]) > offset) {
				carIdx = lane[0];
				offset = Car::getCarOffset(lane[0]);
			}
		} else {
			// 当前检索到的第一优先级为非优先车
			if (cars[lane[0]].prior or Car::getCarOffset(lane[0]) > offset) {
				carIdx = lane[0];
				offset = Car::getCarOffset(carIdx);
				prior = cars[carIdx].prior;
			}
		}
	}
	return offset != -1;
}

bool Scheduler::conflict(int carIdx, int direction, const Cross& cross) {
	if (conflict(carIdx, direction, (direction + 1)%4, cross))
		return true;
	if (conflict(carIdx, direction, (direction + 2)%4, cross))
		return true;
	return conflict(carIdx, direction, (direction + 3)%4, cross);
}

bool Scheduler::conflict(int carIdx, int direction, int otherDir, const Cross& cross) {
	assert(direction != otherDir);
	assert(Car::getNextRoad(carIdx) != NOT_DECIDED);
	assert(Car::getNowRoad(carIdx) == cross.roads[direction]);

	int roadId = cross.roads[otherDir];
	if (roadId == -1)
		return false;
	int roadIdx = graph.getRoadIdx(roadId);
	vector<deque<int>>* lanes = inOutLanes(roadIdx, cross.id, true);
	if (lanes == nullptr)
		return false;
	int firstCarIdx = -1;
	if (not getCarFromSequeue(*lanes, firstCarIdx))
		return false;
	
	assert(Car::getNextRoad(firstCarIdx) != NOT_DECIDED);
	if (cars[carIdx].prior and (not cars[firstCarIdx].prior))
		return false;
	if ((not cars[carIdx].prior) and cars[firstCarIdx].prior) {
		int nextRoad1, nextRoad2;
		if (Car::getNextRoad(carIdx) == DESTINATION and Car::getNextRoad(firstCarIdx) == DESTINATION)
			return false;
		if (Car::getNextRoad(carIdx) == DESTINATION)
			nextRoad1 = cross.roads[(direction + 2)%4];
		else 
			nextRoad1 = Car::getNextRoad(carIdx);
		
		if (Car::getNextRoad(firstCarIdx) == DESTINATION)
			nextRoad2 = cross.roads[(otherDir + 2)%4];
		else 
			nextRoad2 = Car::getNextRoad(firstCarIdx);

		return nextRoad1 == nextRoad2;
	}
	if (Car::getNextRoad(carIdx) == DESTINATION or Car::getNextRoad(carIdx) == cross.roads[(direction + 2)%4])
		return false;
	if (Car::getNextRoad(carIdx) == cross.roads[(direction + 1)%4]) {
		if (otherDir == ((direction + 3)%4))
			return Car::getNextRoad(firstCarIdx) == DESTINATION or
					Car::getNextRoad(firstCarIdx) == Car::getNextRoad(carIdx);
		return false;
	}
	if (Car::getNextRoad(carIdx) == cross.roads[(direction + 3)%4]) {
		if (otherDir == ((direction + 1)%4))
			return Car::getNextRoad(firstCarIdx) == DESTINATION or
					Car::getNextRoad(firstCarIdx) == Car::getNextRoad(carIdx);
		if (otherDir == ((direction + 2)%4))
			return Car::getNextRoad(firstCarIdx) == Car::getNextRoad(carIdx);
		return false;
	}
	assert(false);
	return true;
}

bool Scheduler::getChannel(const vector<deque<int>>& lanes, int& channel) {
	for (int i = 0; i < (int)lanes.size(); ++i) {
		if (lanes[i].empty()) {
			channel = i;
			return true;
		}
		int carIdx = lanes[i].back();
		if (Car::getState(carIdx) == WAITING or
			(Car::getState(carIdx) == STOP and Car::getCarOffset(carIdx) > 1)) {
			channel = i;
			return true;		
		}
	}
	return false;
}

bool Scheduler::moveToNextRoad(int carIdx, int nowRoadIdx, int crossId, deque<int>& nowLane) {
	if (Car::getNextRoad(carIdx) == DESTINATION) {
		assert(not nowLane.empty() and nowLane[0] == carIdx);
		nowLane.pop_front();
		if (Car::getState(carIdx) == WAITING)
			--waiting;
		Car::getState(carIdx) = STOP;
		Car::getCarLocation(carIdx) = END;
		cars[carIdx].reachTime = curTime;
		++end; --way;
		if (cars[carIdx].preset)
			presetWay--;
		if (cars[carIdx].prior)
			priorWay--;
		canGoCar.erase(carIdx);
		return true;
	}
	
	int nextRoadIdx = graph.getRoadIdx(Car::getNextRoad(carIdx));
	vector<deque<int>>* nextLanes = inOutLanes(nextRoadIdx, crossId, false);
	assert(nextLanes != nullptr);
	int remain = graph.roads[nowRoadIdx].length - Car::getCarOffset(carIdx);
	int nextRoadSpeed = min(graph.roads[nextRoadIdx].speedLimit, cars[carIdx].maxSpeed);
	int channel = -1;
	if (not getChannel(*nextLanes, channel) or remain >= nextRoadSpeed) {
		Car::getCarOffset(carIdx) = graph.roads[nowRoadIdx].length;
		if (Car::getState(carIdx) == WAITING)
			--waiting;
		Car::getState(carIdx) = STOP;
		return true;
	}
	if (not (*nextLanes)[channel].empty()) {
		int preCarIdx = (*nextLanes)[channel].back();
		if ((Car::getCarOffset(preCarIdx) <= nextRoadSpeed - remain) and (Car::getState(preCarIdx) == WAITING))
			return false;
	}

	assert(not nowLane.empty() and nowLane[0] == carIdx);
	nowLane.pop_front();
	
	int offset = -1;
	if (not (*nextLanes)[channel].empty()) {
		int preCarIdx = (*nextLanes)[channel].back();
		offset = min(nextRoadSpeed - remain, Car::getCarOffset(preCarIdx) - 1);
	} else {
		offset = nextRoadSpeed - remain;
	}

	(*nextLanes)[channel].push_back(carIdx);
	if (Car::getState(carIdx) == WAITING)
		--waiting;

	Car::getState(carIdx) = STOP;
	Car::getCarLaneIdx(carIdx) = channel;
	Car::getCarOffset(carIdx) = offset;
	Car::getFromCross(carIdx) = crossId;
	Car::getToCross(carIdx) = (graph.roads[nextRoadIdx].startId == crossId)? 
						graph.roads[nextRoadIdx].endId : graph.roads[nextRoadIdx].startId;
	Car::getNowRoad(carIdx) = Car::getNextRoad(carIdx);
	++Car::getNowRoadIdx(carIdx);
	Car::getNextRoad(carIdx) = NOT_DECIDED;
	return true;
}


void Scheduler::driveCarInitList(bool prior) {
	for (int i = 0; i < (int)graph.roads.size(); ++i) {
		runCarInInitList(i, prior, true);
		runCarInInitList(i, prior, false);
	}	
}

void Scheduler::runCarInInitList(int roadIdx, bool prior, bool forward) {
	deque<int>* waiting = nullptr;
	if (forward)
		waiting = &network[roadIdx].forWait;
	else
		waiting = &network[roadIdx].backWait;
	for (int idx : *waiting) {
		if ((not cars[idx].prior) and prior)
			break;
		if (Car::getCarLocation(idx) != HOME)
			continue;
		runToRoad(idx);
	}
}

vector<deque<int>>* Scheduler::inOutLanes(int roadIdx, int crossId, bool in) {
	vector<deque<int>>* lanes = nullptr; 
	if (graph.roads[roadIdx].startId == crossId) {
		if (in) {
			if (graph.roads[roadIdx].duplex)
				lanes = &network[roadIdx].backward;
		} else {
			lanes = &network[roadIdx].forward;
		}
	} else if (graph.roads[roadIdx].endId == crossId) {
		if (in) {
			lanes = &network[roadIdx].forward;
		} else {
			if (graph.roads[roadIdx].duplex)
				lanes = &network[roadIdx].backward;
		}
	} else {
		cout << graph.roads[roadIdx].startId << " " << graph.roads[roadIdx].endId
			<< " " << crossId << endl;
		assert(false);
	}
	return lanes;
}


bool Scheduler::runToRoad(int carIdx) {
	assert(Car::getNextRoad(carIdx) != NOT_DECIDED and Car::getNextRoad(carIdx) != DESTINATION);

	int newRoadIdx = graph.getRoadIdx(Car::getNextRoad(carIdx));
	vector<deque<int>>* nextLane = inOutLanes(newRoadIdx, cars[carIdx].src, false);
	assert(nextLane != nullptr);
	int newLaneIdx = -1;
	if (not getChannel(*nextLane, newLaneIdx))
		return false;
	
	int nextRoadSpeed = min(graph.roads[newRoadIdx].speedLimit, cars[carIdx].maxSpeed);
	if (not (*nextLane)[newLaneIdx].empty()) {
		int preCarIdx = (*nextLane)[newLaneIdx].back();
		if (Car::getCarOffset(preCarIdx) <= nextRoadSpeed and Car::getState(preCarIdx) == WAITING) {
			return false;
		}
	}
	int offset = -1;
	if (not (*nextLane)[newLaneIdx].empty()) {
		int preCarIdx = (*nextLane)[newLaneIdx].back();
		offset = min(nextRoadSpeed, Car::getCarOffset(preCarIdx) - 1);
	} else {
		offset = nextRoadSpeed;
	}
	++way; --home;
	if (cars[carIdx].preset)
		presetWay++;
	if (cars[carIdx].prior)
		priorWay++;
	(*nextLane)[newLaneIdx].push_back(carIdx);

	Car::getCarOffset(carIdx) = offset;
	cars[carIdx].goTime = curTime;
	Car::getState(carIdx) = STOP;
	Car::getCarLocation(carIdx) = ROAD;
	Car::getFromCross(carIdx) = cars[carIdx].src;
	Car::getToCross(carIdx) = (cars[carIdx].src == graph.roads[newRoadIdx].startId)?
						graph.roads[newRoadIdx].endId : graph.roads[newRoadIdx].startId;
	Car::getCarLaneIdx(carIdx) = newLaneIdx;
	Car::getNowRoad(carIdx) = Car::getNextRoad(carIdx);
	Car::getNowRoadIdx(carIdx) = 0;
	Car::getNextRoad(carIdx) = NOT_DECIDED;
	return true;

}

void Scheduler::initWaitList() {
	for (RoadSimulator &road : network) {
		road.backWait.clear();
		road.forWait.clear();
	}

	vector<int> availCarList;
	int curJ = 0;
	if (not sorted and garageSize < 10000) {
		sort(garageCarList.begin(), garageCarList.begin() + garageSize, [this](const int& idx1, const int& idx2)->bool { return this->cars[idx1].maxSpeed < this->cars[idx2].maxSpeed; });
		sorted = true;
	}
	for (int j = 0; j < garageSize; j++) {
//	for (int i = 0; i < (int)cars.size(); ++i) {
		int i = garageCarList[j];
		if (not cars[i].preset and Car::getCarLocation(i) == HOME /*cars[i].startTime == NOT_DECIDED*/ and
			curTime >= cars[i].planTime) {
			if (readyToGo(i)) {
				cars[i].startTime = curTime;
			} else {
				cars[i].route.clear();
				cars[i].startTime = NOT_DECIDED;
			}
		}
		if (Car::getCarLocation(i) == HOME and cars[i].startTime <= curTime) {
			if (decide(i)) {
				canGoCar.emplace(i);
				availCarList.emplace_back(i);
			} else {
				assert(not cars[i].preset and not cars[i].reset);
				cars[i].route.clear();
				cars[i].startTime = NOT_DECIDED;
			}
		} 
		if (Car::getCarLocation(i) == HOME) {
			garageCarList[curJ++] = garageCarList[j];
		}
	}
	garageSize = curJ;
	for (int carIdx : availCarList) {
		assert (Car::getNextRoad(carIdx) != NOT_DECIDED);
		int nextRoadIdx = graph.getRoadIdx(Car::getNextRoad(carIdx));
		if (cars[carIdx].src == graph.roads[nextRoadIdx].startId) {
			network[nextRoadIdx].forWait.emplace_back(carIdx);
		} else if (cars[carIdx].src == graph.roads[nextRoadIdx].endId) {
			network[nextRoadIdx].backWait.emplace_back(carIdx);
		} else {
			assert(false);
		}
	}

	auto lambda = [this](int idx1, int idx2)->bool {
		if (this->cars[idx1].prior and not this->cars[idx2].prior)
			return true;
		if (not this->cars[idx1].prior and this->cars[idx2].prior)
			return false;
		if (this->cars[idx1].startTime < this->cars[idx2].startTime)
			return true;
		if (this->cars[idx1].startTime > this->cars[idx2].startTime)
			return false;
		return this->cars[idx1].id < this->cars[idx2].id;
	};
	for (RoadSimulator &road : network) {
		sort(road.forWait.begin(), road.forWait.end(), lambda);
		sort(road.backWait.begin(), road.backWait.end(), lambda);
	}
}

void Scheduler::updatePenalty() {
	const double stride = 0.1;
	vector<int> penalty(graph.roads.size(), 0);
	for (int carIdx = 0; carIdx < (int)cars.size(); ++carIdx) {
		if (Car::getState(carIdx) == WAITING) {
			int roadIdx = graph.getRoadIdx(Car::getNowRoad(carIdx));
			graph.roads[roadIdx].penalty += stride;
			++penalty[roadIdx];
		}
	}
	/*
	int maxIdx = -1, maxPenalty = 0;
	for (int i = 0; i < (int)penalty.size(); ++i)
		maxIdx = (penalty[i] > maxPenalty)? i : maxIdx;
	graph.roads[maxIdx].penalty *= 10;
	 */
}


void Scheduler::display() {
	cout << "t = " << curTime << ": " << home << " cars at home, " << way << " cars on the road, " 
		<< end << " cars have reached destination, " << priorWay << " prior cars and " << presetWay << " preset cars on the road."  << endl;
}

void Scheduler::displayWaitingCars() {
	for (int i = 0; i < (int)cars.size(); ++i) {
		Car &car = cars[i];
		if (Car::getCarLocation(i) == ROAD and Car::getState(i) == WAITING) {
			cout << car.id << " " << Car::getNowRoad(i) << " " << Car::getFromCross(i) << " " << Car::getToCross(i)
				<< " " <<  Car::getCarLocation(i) << " " << Car::getCarOffset(i) << endl;
		}
	}
}

void Scheduler::displayCarById(int id) {
	for (int i = 0; i < (int)cars.size(); ++i) {
		Car &car = cars[i];
		if (car.id == id) {
			cout << "id: " << car.id << " laneIdx: " << Car::getCarOffset(i)
				<< " offset: " << Car::getCarOffset(i) << " maxSpeed: " << car.maxSpeed << " ";
			switch (Car::getState(i)) {
				case READY: cout << "READY "; break;
				case WAITING: cout << "WAITING "; break;
				case STOP: cout << "STOP "; break;
				default: assert(false);
			}
			cout << "now: "<< Car::getNowRoad(i) << " next: " << Car::getNextRoad(i) << endl;
		}
	}
}

void Scheduler::displayRoadById(int id) {
	auto it = graph.roadIdx.find(id);
	assert(it != graph.roadIdx.end());
	int idx = it->second;
	cout << "id: " << graph.roads[idx].id << " length: " << graph.roads[idx].length << " " 
		<< " laneNumber: " << graph.roads[idx].laneNumber << " " << graph.roads[idx].duplex << endl;
	cout << "forward: " << endl;
	for (int i = 0; i < (int)network[idx].forward.size(); ++i) {
		cout << "lane " << i << ": ";
		for (int carIdx : network[idx].forward[i])
			cout << cars[carIdx].id << " ";
		cout << endl;
	}
	cout << "backward: " << endl;
	for (int i = 0; i < (int)network[idx].backward.size(); ++i) {
		cout << "lane " << i << ": ";
		for (int carIdx : network[idx].backward[i])
			cout << cars[carIdx].id << " ";
		cout << endl;
	}
	cout << "----------------------------------" << endl;
}


void Scheduler::readAnswer(ifstream& answerIn) {
	string line;
	while (getline(answerIn, line)) {
		if (line.empty() or line[0] == '#')
			continue;

		for (char &ch : line) {
			if (ch == '(' or ch == ')' or ch == ',')
				ch = ' ';
		}

		stringstream ss;
		ss << line;
		int carId, startTime;
		ss >> carId >> startTime;
		
		int carIdx = getCarIdx(carId);
		if (cars[carIdx].preset)
			continue;

		cars[carIdx].startTime = startTime;
		int roadId;
		while (ss >> roadId) {
			cars[carIdx].route.emplace_back(roadId);
		}
	}
}

void Scheduler::outputLog() {
	cout << "timeSlice=" << curTime << endl;
	for (int i = 0; i < (int)graph.roads.size(); ++i) {
		cout << "id: " << graph.roads[i].id << " from: " << graph.roads[i].startId << " to: " << graph.roads[i].endId
			 << " laneNumber: " << graph.roads[i].laneNumber << " speed: " << graph.roads[i].speedLimit 
			 << " length: " << graph.roads[i].length << endl;
		cout << "forward: " << endl;
		for (int j = 0; j < (int)network[i].forward.size(); ++j) {
			cout << "lane " << j << ": " << endl;
			for (int carIdx : network[i].forward[j]) {
				if (Car::getState(carIdx) != WAITING)
					continue;
				cout << "id: " << cars[carIdx].id << " position: " << Car::getCarOffset(carIdx)
					<< " speed: " << cars[carIdx].maxSpeed << " start time: " << cars[carIdx].startTime
					<< " src: " << cars[carIdx].src << " dest: " << cars[carIdx].dest;
				if (cars[carIdx].preset)
					cout << " preset";
				cout << endl;
			} 
		}
		cout << "backward: " << endl;
		for (int j = 0; j < (int)network[i].backward.size(); ++j) {
			cout << "lane " << j << ": " << endl;
			for (int carIdx : network[i].backward[j]) {
				if (Car::getState(carIdx) != WAITING)
					continue;
				cout << "id: " << cars[carIdx].id << " position: " << Car::getCarOffset(carIdx)
					<< " speed: " << cars[carIdx].maxSpeed << " start time: " << cars[carIdx].startTime
					<< " src: " << cars[carIdx].src << " dest: " << cars[carIdx].dest;
				if (cars[carIdx].preset)
					cout << " preset";
				cout << endl;
			} 
		}
		cout << "--------------------------" << endl;
	}
	cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << endl;
}

void Scheduler::outputStatus() {
	if (curTime == 0)
		return;
	ofstream status;
	status.open("road_log.txt", ios::out|ios::app);
	if (status.fail()) {
		cout << "open road_log.txt failed" << endl;
		assert(false);
	}
	for (int i = 0; i < (int)network.size(); ++i) {
		int length = graph.roads[i].length;
		for (int j = 0; j < (int)network[i].forward.size(); ++j) {
			status << graph.roads[i].id << "," << curTime << "," << j << ",";
			for (int carIdx : network[i].forward[j])
				status << length - Car::getCarOffset(carIdx) << "," << cars[carIdx].id << ",";
			status << endl;
		}
		if (graph.roads[i].duplex) {
			for (int j = 0; j < (int)network[i].backward.size(); ++j) {
				status << graph.roads[i].id << "," << curTime << "," << j << ",";
				for (int carIdx : network[i].backward[j])
					status << length - Car::getCarOffset(carIdx) << "," << cars[carIdx].id << ",";
				status << endl;
			}
		}
	}
}