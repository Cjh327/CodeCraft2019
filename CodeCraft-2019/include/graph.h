#ifndef __GRAPH_H__
#define __GRAPH_H__

#include "common.h"


struct Road {
	int id;
	int length;
	int speedLimit;
	int laneNumber;
	int startId;
	int endId;
	bool duplex;
	
	int forJam;
	int backJam;
	bool keyRoad;
	int extraJam;

	int forPresetJam;
	int backPresetJam;

	double penalty;
};

struct Cross {
	int id;
	vector<int> roads;
	int waitCarNum;

	int x, y;
	bool edge;
	int gapNum;
};

class Car;

class Graph {
	vector<Cross> crosses;
	vector<Road> roads;
	int totalCapacity;
	unordered_map<int, int> crossIdx;
	unordered_map<int, int> roadIdx;

	/*
	 * hash为从(crossId, crossId)二元组到
	 * 连接这两个cross的道路索引即idx的映射
	 * */
	map<pair<int, int>, int> hash;


	double getRoadWeight(const Road&, const Car&, bool);
	double getRoadFloydWeight(const Road&, bool);

public:
	
	friend class Scheduler;
	friend class Simulator;

	Graph(ifstream&, ifstream&);
	void displayRoads();
	void displayCrosses();
	pair<vector<vector<double>>, vector<vector<int>>> floyd();
	int dijkstra(Car&, int, int, int);

	vector<int> prevToRoute(const vector<int>&, int, int);
	vector<int> dijkstraForPrior(Car&);

	void setKeyRoad();

	Road& getRoadById(int id);
	Cross& getCrossById(int id);

	int getRoadIdx(int id);
	int getCrossRoadIdx(int crossId1, int crossId2);
	int getCrossIdx(int id);

    bool isRoadCongested(int, int);
	int getCrossKind(int, int);

	vector<vector<int>> naiveFloyd();
	void detectEdge();
};

#endif