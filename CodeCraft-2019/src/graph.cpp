#include "graph.h"
#include "scheduler.h"
#include <cmath>

/*
 *  读入roadStream与crossStream文件流中内容
 *  roads:
 *  	(道路id，道路长度，最高限速，车道数目，起始点id，终点id，是否双向)
 *  crosses:
 * 		(结点id，道路id，道路id，道路id，道路id)
 * 		从北方道路开始顺时钟方向
 */


Graph::Graph(ifstream& roadStream, ifstream& crossStream) {
	totalCapacity = 0;
	string str;
	while (getline(roadStream, str)) {
		if (str.empty() or str[0] == '#')
			continue;
		int duplex;
		Road road;
		sscanf(str.c_str(), "(%d, %d, %d, %d, %d, %d, %d)", &road.id, &road.length, &road.speedLimit,
			&road.laneNumber, &road.startId, &road.endId, &duplex);
		road.duplex = (duplex == 1);
		road.forJam = road.backJam = road.extraJam = 0;
		road.penalty = 0;
		road.keyRoad = false;
		if (road.duplex) {
			totalCapacity += 2 * road.length * road.laneNumber;
		} else {
			totalCapacity += road.length * road.laneNumber;
		}

		hash.emplace(make_pair(road.startId, road.endId), roads.size());
		if (road.duplex)
			hash.emplace(make_pair(road.endId, road.startId), roads.size());
		roads.emplace_back(road);
	}

	sort(roads.begin(), roads.end(), [](const Road& r1, const Road& r2)->bool { return r1.id < r2.id; });
	for (int i = 0; i < (int)roads.size(); ++i)
		roadIdx.emplace(roads[i].id, i);

	while (getline(crossStream, str)) {
		if (str.empty() or str[0] == '#')
			continue;
		Cross cross;
		int pathes[4];
		sscanf(str.c_str(), "(%d, %d, %d, %d, %d)", &cross.id, &pathes[0], &pathes[1], &pathes[2], &pathes[3]);
		for (int i = 0; i < 4; ++i) {
			cross.roads.emplace_back(pathes[i]);
		}
		cross.waitCarNum = 0;
		cross.gapNum = 0;
		crosses.emplace_back(cross);
	}

	// crosses中所有cross以id升序排列
	sort(crosses.begin(), crosses.end(), [](const Cross& c1, const Cross& c2)->bool { return c1.id < c2.id; });
	for (int i = 0; i < (int)crosses.size(); ++i)
		crossIdx.emplace(crosses[i].id, i);
	//setKeyRoad();

	detectEdge();
}

void Graph::displayRoads() {
	for (Road &road : roads) {
		printf("(%d, %d, %d, %d, %d, %d, %d)\n", road.id, road.length, road.speedLimit,
			road.laneNumber, road.startId, road.endId, road.duplex);
	}
}

void Graph::displayCrosses() {
	for (Cross &cross : crosses) {
		assert(not cross.roads.empty());
		printf("(%d: %d", cross.id, cross.roads[0]);
		for (int i = 1; i < (int)cross.roads.size(); ++i)
			printf(", %d", cross.roads[i]);
		printf(")\n");
	}
}

Road& Graph::getRoadById(int id) {
	auto it = roadIdx.find(id);
	assert(it != roadIdx.end());
	return roads[it->second];
}

Cross& Graph::getCrossById(int id) {
	auto it = crossIdx.find(id);
	assert(it != crossIdx.end());
	return crosses[it->second];
}

int Graph::getRoadIdx(int id) {
	auto it = roadIdx.find(id);
	assert(it != roadIdx.end());
	return it->second;
}

int Graph::getCrossRoadIdx(int crossId1, int crossId2) {
	auto it = hash.find(make_pair(crossId1, crossId2));
	assert(it != hash.end());
	return it->second;
}

int Graph::getCrossIdx(int id) {
	auto it = crossIdx.find(id);
	assert(it != crossIdx.end());
	return it->second;
}


int Graph::dijkstra(Car& car , int startCrossId, int endCrossId, int blockRoadId) {
	//cout << car.id << " " << startCrossId << " " << endCrossId << " " << blockRoadId << endl;
	
	const int inf = 0x3f3f3f3f;
	const int size = (int)crosses.size();

	vector<int> prev(size, -1);
	vector<double> dist(size, inf);
	vector<bool> used(size, false);

	auto it1 = crossIdx.find(startCrossId), it2 = crossIdx.find(endCrossId);
	assert(it1 != roadIdx.end() and it2 != roadIdx.end());
	int startCrossIdx = it1->second, endCrossIdx = it2->second;
	dist[startCrossIdx] = 0;
	used[startCrossIdx] = true;
	for (int roadId : crosses[startCrossIdx].roads) {
		if (roadId == -1 or roadId == blockRoadId)
			continue;
		auto it = roadIdx.find(roadId);
		assert(it != roadIdx.end());
		int rIdx = it->second;
		Road road = roads[rIdx];
		int idx = crossIdx[road.endId];
		if (crossIdx[road.startId] != startCrossIdx && road.duplex) {
			assert(crossIdx[road.endId] == startCrossIdx);
			idx = crossIdx[road.startId];
			double weight = getRoadWeight(road, car, false);
			dist[idx] = weight;
			prev[idx] = startCrossIdx;
		} else if (crossIdx[road.startId] == startCrossIdx) {
			double weight = getRoadWeight(road, car, true);
			dist[idx] = weight;
			prev[idx] = startCrossIdx;
		}
	}

	for (int i = 1; i < size; i++) {
		int minDist = inf;
		int minIdx = -1;
		for (int j = 0; j < size; j++) {
			if (!used[j] && dist[j] < minDist) {
				minDist = dist[j];
				minIdx = j;
			}
		}
		if (minIdx == endCrossIdx) {
			int tmpCrossIdx = endCrossIdx;
			while (prev[tmpCrossIdx] != startCrossIdx) {
				tmpCrossIdx = prev[tmpCrossIdx];
			}
			auto iter = hash.find(make_pair(crosses[startCrossIdx].id, crosses[tmpCrossIdx].id)); 
			assert(iter != hash.end());
	//		cout << crosses[startCrossIdx].id << " " << crosses[tmpCrossIdx].id << roads[iter->second].id << endl;
			return roads[iter->second].id;
		}
		assert(minIdx >= 0);
		used[minIdx] = true;
		for (int roadId : crosses[minIdx].roads) {
			if (roadId == -1)
				continue;
			auto it = roadIdx.find(roadId);
			assert(it != roadIdx.end());
			int rIdx = it->second;
			Road road = roads[rIdx];
			int idx = crossIdx[road.endId];
			if (crossIdx[road.startId] != minIdx && road.duplex) {
				assert(crossIdx[road.endId] == minIdx);
				idx = crossIdx[road.startId];
				double weight = getRoadWeight(road, car, false);
				if (dist[idx] > dist[minIdx] + weight) {
					dist[idx] = dist[minIdx] + weight;
					prev[idx] = minIdx;
				}
			} else if (crossIdx[road.startId] == minIdx) {
				double weight = getRoadWeight(road, car, true);
				if (dist[idx] > dist[minIdx] + weight) {
					dist[idx] = dist[minIdx] + weight;
					prev[idx] = minIdx;
				}
			}
		}
	}
	for (int i = 0; i < size; i++) {
		assert(used[i]);
	}
	int tmpCrossIdx = endCrossIdx;
	while (prev[tmpCrossIdx] != startCrossIdx) {
		tmpCrossIdx = prev[tmpCrossIdx];
	}
	auto iter = hash.find(make_pair(crosses[startCrossIdx].id, crosses[tmpCrossIdx].id));
	//if (iter == hash.end()) {
	//	cout << crosses[startCrossIdx].id << " " << crosses[tmpCrossIdx].id << endl;
	//}
	assert(iter != hash.end());
	return roads[iter->second].id;
}

pair<vector<vector<double>>, vector<vector<int>>> Graph::floyd() {
	vector<vector<int>> next(crosses.size(), vector<int>(crosses.size(), -1));
	vector<vector<double>> dist(crosses.size(), vector<double>(crosses.size(), 0x3f3f3f3f));
	for (Road &road : roads) {
		auto it1 = crossIdx.find(road.startId), it2 = crossIdx.find(road.endId);
		assert(it1 != roadIdx.end() and it2 != roadIdx.end());
		dist[it1->second][it2->second] = getRoadFloydWeight(road, true);;
		next[it1->second][it2->second] = it2->second;
		if (road.duplex) {
			dist[it2->second][it1->second] = getRoadFloydWeight(road, false);
			next[it2->second][it1->second] = it1->second;
		}
	}
	for (int k = 0; k < (int)crosses.size(); ++k) {
		for (int i = 0; i < (int)crosses.size(); ++i) {
			for (int j = 0; j < (int)crosses.size(); ++j) {
				if (dist[i][k] + dist[k][j] < dist[i][j]) {
					dist[i][j] = dist[i][k] + dist[k][j];
					next[i][j] = next[i][k];
				}
			}
		}
	}
	for (auto v : dist) {
		for (int x : v) {
			assert(x < 0x3f3f3f3f);
		}
	}
	return make_pair(dist, next);
}

void Graph::setKeyRoad() {
	vector<pair<int, int>> roadOccur(roads.size(), pair<int, int>(0, 0)); // idx, occur times
	for (int i = 0; i < (int)roads.size(); i++) {
		roadOccur[i].first = i;
	}
	vector<vector<int>> next = floyd().second;
	for (int i = 0; i < (int)next.size(); i++) {
		for (int j = 0; j < (int)next[i].size(); j++) {
			int startIdx = i, nextIdx = next[i][j];
			auto it = hash.find(make_pair(crosses[startIdx].id, crosses[nextIdx].id));
			assert(it != hash.end());
			roadOccur[it->second].second++;
		}
	}
	sort(roadOccur.begin(), roadOccur.end(), [](const pair<int,int>& c1, const pair<int,int>& c2)->bool { return c1.second > c2.second; });
	for (int i = 0; i < (int)roadOccur.size() / 10; i++) {
		int idx = roadOccur[i].first;
		roads[idx].keyRoad = true;
		cout << roads[idx].id << endl;
	}
}

vector<int> Graph::prevToRoute(const vector<int>& prev, int src, int dest) {
	auto it1 = crossIdx.find(src), it2 = crossIdx.find(dest);
	assert(it1 != crossIdx.end() and it2 != crossIdx.end());
	int startIdx = it1->second, endIdx = it2->second;
	vector<int> route;
	while (endIdx != startIdx) {
		int prevIdx = prev[endIdx];
		auto it = hash.find(make_pair(crosses[prevIdx].id, crosses[endIdx].id));
		assert(it != hash.end());
		int roadId = roads[it->second].id;
		route.insert(route.begin(), roadId);
		endIdx = prevIdx;
	}
	return route;
}

vector<int> Graph::dijkstraForPrior(Car& car) {
	const int inf = 0x3f3f3f3f;
	const int size = crosses.size();
	const int startId = car.src, endId = car.dest;

	vector<int> prev(size, -1);
	vector<double> dist(size, inf);
	vector<bool> used(size, false);

	auto it1 = crossIdx.find(startId);
	assert(it1 != roadIdx.end());
	int startIdx = it1->second;
	auto it2 = crossIdx.find(endId);
	assert(it2 != roadIdx.end());
	int endIdx = it2->second;
	dist[startIdx] = 0;
	used[startIdx] = true;
	for (int roadId : crosses[startIdx].roads) {
		if (roadId == -1)
			continue;
		auto it = roadIdx.find(roadId);
		assert(it != roadIdx.end());
		int rIdx = it->second;
		Road road = roads[rIdx];
		int idx = crossIdx[road.endId];
		if (crossIdx[road.startId] != startIdx && road.duplex) {
			assert(crossIdx[road.endId] == startIdx);
			idx = crossIdx[road.startId];
			double weight = getRoadWeight(road, car, false);
			dist[idx] = weight;
			prev[idx] = startIdx;
		} else if (crossIdx[road.startId] == startIdx) {
			double weight = getRoadWeight(road, car, true);
			dist[idx] = weight;
			prev[idx] = startIdx;
		}
	}

	for (int i = 1; i < size; i++) {
		double minDist = inf;
		int minIdx = -1;
		for (int j = 0; j < size; j++) {
			if (!used[j] && dist[j] < minDist) {
				minDist = dist[j];
				minIdx = j;
			}
		}
		if (minIdx == endIdx) {
			return prevToRoute(prev, car.src, car.dest);
		}
		assert(minIdx >= 0);
		used[minIdx] = true;
		for (int roadId : crosses[minIdx].roads) {
			if (roadId == -1)
				continue;
			auto it = roadIdx.find(roadId);
			assert(it != roadIdx.end());
			int rIdx = it->second;
			Road road = roads[rIdx];
			int idx = crossIdx[road.endId];
			if (crossIdx[road.startId] != minIdx && road.duplex) {
				assert(crossIdx[road.endId] == minIdx);
				idx = crossIdx[road.startId];
				double weight = getRoadWeight(road, car, false);
				if (dist[idx] > dist[minIdx] + weight) {
					dist[idx] = dist[minIdx] + weight;
					prev[idx] = minIdx;
				}
			} else if (crossIdx[road.startId] == minIdx) {
				double weight = getRoadWeight(road, car, true);
				if (dist[idx] > dist[minIdx] + weight) {
					dist[idx] = dist[minIdx] + weight;
					prev[idx] = minIdx;
				}
			}
		}
	}
	for (int i = 0; i < size; i++) {
		assert(used[i]);
	}
	return prevToRoute(prev, car.src, car.dest);
}

bool Graph::isRoadCongested(int roadId, int curCrossIdx) {
	return getRoadById(roadId).penalty > 15;
	Road road = roads[roadIdx[roadId]];
	bool isForward = (crosses[curCrossIdx].id == road.startId);
	int jamDegree = isForward ? road.forJam : road.backJam;
	//cout <<(double)jamDegree / (road.laneNumber * road.length)<<endl; 
	return ((double)jamDegree / (road.laneNumber * road.length)) > 0.6;
}

int Graph::getCrossKind(int crossId, int fromRoadId) {
	int crossIdx = getCrossIdx(crossId);
	Cross cross = crosses[crossIdx];
	int numRoad = 0;
	if (getCrossById(crossId).edge) {
		return 2;
	}
	for (int roadId : cross.roads) {
		if (roadId != -1) {
			numRoad++;
		}
	}
	assert(numRoad <= 4);
	if (numRoad == 3) {
		int idx = -1;
		for (int i = 0; i < (int)cross.roads.size(); ++i) {
			if (cross.roads[i] == fromRoadId) {
				idx = i;
				break;
			}
		}
		assert(idx != -1);
		if (cross.roads[(idx + 2)%4] == -1) {
			return 1;
		}
	}
	return 0;
}

double Graph::getRoadWeight(const Road& road, const Car& car, bool forward) {
	double k = 100;
	double l = road.length, m = road.laneNumber, vm = road.speedLimit, d = forward ? road.forJam : road.backJam;
	double v = car.maxSpeed;
	int crossId = forward ? road.endId : road.startId;
	int crossKind = getCrossKind(crossId, road.id);
	double p = (forward ? road.forPresetJam : road.backPresetJam);

	if (crossKind == 1) {
		k *= 2;
		if (d / (l * m) > 0.8) {
			k *= 4;
		}
	} else if ((d + p) / (l * m) > 0.8) {
		k *= 6;
	} else if (d / (l * m) > 0.8) {
		k *= 4;
	}
	else {
	//	k += p;
	}

	double weight = 0;
	assert(k > 0 and d >= 0);
	if (car.prior) {
		weight = l / min(vm, v) + 10 * d / (l * m);
	} else {
		weight = l / min(vm, v) + k * (p+d) / (l * m)  + road.penalty;
	}
	return weight;
}

double Graph::getRoadFloydWeight(const Road& road, bool forward) {
	if (not road.duplex and not forward) {
		assert(false);
	}
	double k = 100;
	double l = road.length, m = road.laneNumber, vm = road.speedLimit, d = forward ? road.forJam : road.backJam;
	int crossId = forward ? road.endId : road.startId;
	int crossKind = getCrossKind(crossId, road.id);
	double p = (forward ? road.forPresetJam : road.backPresetJam);

	if (crossKind == 1) {
		k *= 2;
		if (d / (l * m) > 0.8) {
			k *= 4;
		}
	} else if ((d + p) / (l * m) > 0.8) {
		k *= 6;
	} else if (d / (l * m) > 0.8) {
		k *= 4;
	}
	else {
	//	k += p;
	}

	assert(k > 0 and d >= 0);
	double weight = l / vm + k * (p + d)/ (l * m) + road.penalty;
	return weight;
}


void Graph::detectEdge() {
	set<int> explored;
	set<int> visited;
	map<int, int> roadDirection;

	crosses[0].x = 0;
	crosses[0].y = 0;
	assert(not crosses.empty());
	visited.emplace(0);

	for (int i = 0; i < (int)crosses[0].roads.size(); ++i) {
		int roadId = crosses[0].roads[i];
		if (roadId == -1)
			continue;
		int roadIdx = getRoadIdx(roadId);
		int next = -1;
		if (crosses[0].id == roads[roadIdx].startId)
			next = roads[roadIdx].endId;
		else if (crosses[0].id == roads[roadIdx].endId)
			next = roads[roadIdx].startId;
		else
			assert(false);
		int crossIdx = getCrossIdx(next);
		switch (i) {
			case 0: 
				crosses[crossIdx].x = 0;
				crosses[crossIdx].y = 1;
				break;
			case 1: 
				crosses[crossIdx].x = 1;
				crosses[crossIdx].y = 0;
				break;
			case 2: 
				crosses[crossIdx].x = 0;
				crosses[crossIdx].y = -1;
				break;
			case 3: 
				crosses[crossIdx].x = -1;
				crosses[crossIdx].y = 0;
				break;
			default:
				assert(false);
		}
		if (crosses[0].id == roads[roadIdx].startId)
			roadDirection[roadIdx] = i;
		else if (crosses[0].id == roads[roadIdx].endId)
			roadDirection[roadIdx] = (i + 2)%4;
		else
			assert(false);

		if (explored.count(crossIdx) == 0 and visited.count(crossIdx) == 0)
			explored.emplace(crossIdx);
	}

	while (not explored.empty()) {
		auto it = explored.begin();
		int crossIdx = *it;
		visited.emplace(crossIdx);
		explored.erase(it);

		int direction = -1, idx = -1;
		bool inRoad = false;

		for (int i = 0; i < (int)crosses[crossIdx].roads.size(); ++i) {
			int roadId = crosses[crossIdx].roads[i];
			if (roadId == -1)
				continue;
			int roadIdx = getRoadIdx(roadId);
			if (roadDirection.find(roadIdx) == roadDirection.end())
				continue;

			direction = roadDirection[roadIdx];
			if (roads[roadIdx].startId == crosses[crossIdx].id)
				inRoad = false;
			else if (roads[roadIdx].endId == crosses[crossIdx].id)
				inRoad = true;
			else
				assert(false);
			idx = i;
			break;
		}
		assert(idx != -1);

		int direct = -1;
		if ((direction == 0 and not inRoad) or (direction == 2 and inRoad))
                direct = 0;
        else if ((direction == 1 and not inRoad) or (direction == 3 and inRoad))
                direct = 1;
        else if ((direction == 2 and not inRoad) or (direction == 0 and inRoad))
                direct = 2;
        else if ((direction == 3 and not inRoad) or (direction == 1 and inRoad))
                direct = 3;
        else
            assert(false);

		vector<int> directionArray = {-1, -1, -1, -1};
		for (int i = 0; i < (int)crosses[crossIdx].roads.size(); ++i) {
			int roadId = crosses[crossIdx].roads[i];
			if (roadId == -1)
				continue;
			int roadIdx = getRoadIdx(roadId);
			int offset = (i - idx + 4)%4;

			if (roads[roadIdx].startId == crosses[crossIdx].id)
				directionArray[i] = (direct + offset)%4;
			else if (roads[roadIdx].endId == crosses[crossIdx].id)
				directionArray[i] = (direct + offset + 2)%4;
			else
				assert(false);

			roadDirection[roadIdx] = directionArray[i];	
			
			int next = -1;
			if (crosses[crossIdx].id == roads[roadIdx].startId)
				next = roads[roadIdx].endId;
			else if (crosses[crossIdx].id == roads[roadIdx].endId)
				next = roads[roadIdx].startId;
			else
				assert(false);
			int nextCrossIdx = getCrossIdx(next);

			if (explored.count(nextCrossIdx) == 0 and visited.count(nextCrossIdx) == 0) {
				explored.emplace(nextCrossIdx);
				switch ((direct + offset)%4) {
					case 0:
						crosses[nextCrossIdx].x = crosses[crossIdx].x;
						crosses[nextCrossIdx].y = crosses[crossIdx].y + 1;
						break;
					case 1:
						crosses[nextCrossIdx].x = crosses[crossIdx].x + 1;
						crosses[nextCrossIdx].y = crosses[crossIdx].y;
						break;
					case 2:
						crosses[nextCrossIdx].x = crosses[crossIdx].x;
						crosses[nextCrossIdx].y = crosses[crossIdx].y - 1;
						break;
					case 3:
						crosses[nextCrossIdx].x = crosses[crossIdx].x - 1;
						crosses[nextCrossIdx].y = crosses[crossIdx].y;
						break;
					default:
						assert(false);
				}
			}
		}
	}

	int minX, maxX, minY, maxY;
	minX = minY = 0x3f3f3f3f;
	maxX = maxY = -2147483648;
	for (int i = 0; i < (int)crosses.size(); ++i) {
		/*cout << "id: " << crosses[i].id << " ("
			<< crosses[i].x << ","
			<< crosses[i].y << ")" << endl;*/
		crosses[i].edge = false;
		minX = min(crosses[i].x, minX);
		maxX = max(crosses[i].x, maxX);
		minY = min(crosses[i].y, minY);
		maxY = max(crosses[i].y, maxY);
	}
	for (int i = 0; i < (int)crosses.size(); ++i) {
		if (crosses[i].x == minX or crosses[i].x == maxX or
			crosses[i].y == minY or crosses[i].y == maxY)
			crosses[i].edge = true;
	}
	/*
	//cout << maxY << " " << minY  << " " << maxX << " " <<  minX  << endl;
	vector<vector<int>> coords(maxY - minY + 1, vector<int>(maxX - minX + 1, 0));
	int originalX = minX, originalY = minY;
	for (Cross cross : crosses) {
		int coordX = cross.x - originalX;
		int coordY = cross.y - originalY;
		//cout << coordY << " " << coordX << endl;
		coords[coordY][coordX] = cross.id;
	}
	const int width = 3;
	for (int i = 0; i < (int)coords.size(); i++) {
		for (int j = 0; j < (int)coords[i].size(); j++) {
			if (coords[i][j] == 0) {
				for (int ii = max(i-width, 0); ii <= min(i+width, (int)coords.size()-1); ii++) {
					for (int jj = max(j-width, 0); jj <= min(j+width, (int)coords[i].size()-1); jj++) {
						if ((ii == i and jj == j) or coords[ii][jj] == 0) {
							continue;
						}
						getCrossById(coords[ii][jj]).gapNum++;
					}
				}
			}
		}
	}
	*/
	/*
	for (int i = 0; i < (int)coords.size(); i++) {
		for (int j = 0; j < (int)coords[i].size(); j++) {
			cout << coords[i][j] << "\t";
		}
		cout << endl;
	}
	cout << endl;
	for (int i = 0; i < (int)coords.size(); i++) {
		for (int j = 0; j < (int)coords[i].size(); j++) {
			if (coords[i][j] == 0) {
				cout << "0\t";
			} else {
				cout << getCrossById(coords[i][j]).gapNum << "\t";
			}
		}
		cout << endl;
	}
	*/
}

vector<vector<int>> Graph::naiveFloyd() {
	vector<vector<int>> dist(crosses.size(), vector<int>(crosses.size(), 0x3f3f3f3f));
	for (Road &road : roads) {
		int idx1 = getCrossIdx(road.startId), idx2 = getCrossIdx(road.endId);
		dist[idx1][idx2] = road.length;
		if (road.duplex)
			dist[idx2][idx1] = road.length;
	}
	int m = (int)crosses.size();
	for (int k = 0; k < m; ++k) {
		for (int i = 0; i < m; ++i) {
			for (int j = 0; j < m; ++j) {
				if (dist[i][j] > dist[i][k] + dist[k][j])
					dist[i][j] = dist[i][k] + dist[k][j];
			}
		}
	}
	return dist;
}