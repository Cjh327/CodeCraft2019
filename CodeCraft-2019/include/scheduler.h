#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include "common.h"
#include "graph.h"
#include "car.h"

class Scheduler;

struct RoadSimulator {
    vector<deque<int>> forward;
    vector<deque<int>> backward;

    deque<int> backWait;
    deque<int> forWait;

    int forFirstCarIdx;
    int backFirstCarIdx;

    int getForwardJamDegree();
    int getBackwardJamDegree();
    int getForwardPresetJamDegree(Scheduler*);
    int getBackwardPresetJamDegree(Scheduler*);
};

struct FieldInfo {
	// car info
	vector<CarState> infoInstantStates;
	vector<vector<int>> carRoutes;
	// network
	vector<RoadSimulator> infoNetwork;
	// not end car idxs
	vector<int> infoGarageCarList;
	int infoGarageSize;
	// can go car idxs
	set<int> infoCanGoCar;
	// general
	int infoHome, infoWay, infoEnd;
	int infoPresetWay, infoPriorWay;
	int infoWaiting;
	int infoCurTime;
	bool infoSorted;
	int infoGoCarSize;
};


class Scheduler {
public:
    unordered_map<int, int> carIdxes;
    vector<Car> cars;
    vector<int> garageCarList; // idx
    int garageSize;
    vector<int> priorCarIdxs;
    Graph graph;
    vector<RoadSimulator> network;

    vector<vector<double>> dist;
    vector<vector<int>> next;

    int home, way, end;
    int presetWay, priorWay;
    int waiting;
    int curTime;
    bool sorted;

    set<int> canGoCar;

    double a, b;

	vector<FieldInfo> fieldInfoList;        // save every 40 timeslices
	const int upperBound = 20000;
	int goCarSize;
	int stride;

	bool onlyPreset;

public:
    
    friend class Graph;
    Scheduler(ifstream&, ifstream&, ifstream&, ifstream&);
    void outputAnswer(ofstream&);
    void simulate();

    void initNetwork();

    bool taskfinished();
    
    bool run();


    void driveJustCurrentRoad();
    bool driveCarInWaitState();
    void updateRoadCars(vector<deque<int>>& road, int roadIdx);
    void updateRoadCars(vector<deque<int>>& road, int raodIdx, int laneIdx);
    
    bool decide(int carIdx);
    bool readyToGo(int carIdx);

    void updateCrossCars(Cross& cross);
    bool getCarFromSequeue(const vector<deque<int>>& lanes, int& CarIdx);
    bool conflict(int carIdx, int direction, const Cross& cross);
    vector<deque<int>>* inOutLanes(int roadIdx, int crossId, bool in);

    bool conflict(int carIdx, int direction, int otherDir, const Cross& cross);
    bool moveToNextRoad(int carIdx, int nowRoadIdx, int crossId, deque<int>& nowLane);
    bool getChannel(const vector<deque<int>>& lanes, int& channel);
    void driveCarInitList(bool priority);
    void runCarInInitList(int roadIdx, bool priority, bool forward);

    bool runToRoad(int carIdx);
    void initWaitList();

    void display();
    void displayWaitingCars();
    void displayCarById(int id);
    void displayRoadById(int id);

	void outputLog();

    void updateRoads(int);
    void readAnswer(ifstream& answerIn);
    int getCarIdx(int id);

    void computeFactor();
    void updateNextRoadSet();
    void updateRoadJam();

	void outputInfo(int);
    void outputStatus();

	void saveFieldInfo();
	void recoverFieldInfo(int);
	int getRoadAfterNowRoadIdx(int);

	void updatePenalty();
	void changeTenPercent();
};

#endif
