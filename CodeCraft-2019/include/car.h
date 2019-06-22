#ifndef __CAR_H__
#define __CAR_H__

#include "common.h"

#define NOT_DECIDED 0x3f3f3f3f
#define DESTINATION 0x2f2f2f2f
#define INF 0x3f3f3f3f

enum State {READY, WAITING, STOP};
enum Location {HOME, ROAD, END};

struct CarState {
	int from;
	int to;
	int offset;
	int laneIdx;
	enum Location location;
	int nowRoad;
	int nowRoadIdx;
	int waitFor;

	CarState() {
		from = to = offset = laneIdx = nowRoad = -1;
		location = HOME;
		waitFor = NOT_DECIDED;
		nowRoadIdx = -1;
	}
};

class Car {
public:
	static vector<enum State> states;
	static vector<int> nextRoads;
	static vector<CarState> instantStates;
	int id;
	int src;
	int dest;
	int maxSpeed;
	int planTime;
	bool prior;
	bool preset;
	bool reset;

	int startTime;
	int reachTime;
	int goTime;
	vector<int> route;

	explicit Car(vector<int> v): id(v[0]), src(v[1]), dest(v[2]), maxSpeed(v[3]), planTime(v[4]), startTime(INF) {
		prior = v[5] == 1;
		preset = v[6] == 1;
		reachTime = goTime = INF;
		reset = false;
	}

	/**
	 *  location记录小车宏观位置，
	 *  尚未出发、在路上以及已经到达三种状态。
	 *
	 *  state仅当location为在路上时有意义，
	 *  表示当前调度过程中小车微观状态，
	 *  READY: 等待调度
	 *  WATRING: 等待其他车辆调度
	 *  STOP: 调度完成
	 *
	 *  offset表示小车车头离身后路口距离
	 *  laneIdx表示当前车道序号
	 */



	static enum State& getState(int idx) {
		return states[idx];
	}

	static int& getNextRoad(int idx) {
		return nextRoads[idx];
	}

	static int& getFromCross(int idx) {
		return instantStates[idx].from;
	}

	static int& getToCross(int idx) {
		return instantStates[idx].to;
	}

	static int& getCarOffset(int idx) {
		return instantStates[idx].offset;
	}

	static int& getCarLaneIdx(int idx) {
		return instantStates[idx].laneIdx;
	}

	static enum Location& getCarLocation(int idx) {
		return instantStates[idx].location;
	}

	static int& getNowRoad(int idx) {
		return instantStates[idx].nowRoad;
	}

	static int& getNowRoadIdx(int idx) {
		return instantStates[idx].nowRoadIdx;
	}

	static void initState(size_t sz) {
		states.assign(sz, READY);
		nextRoads.assign(sz, NOT_DECIDED);
		CarState initialCarState;
		instantStates.assign(sz, initialCarState);
	}

	static void freshState(size_t sz) {
		memset(&states[0], 0, sizeof(enum State)*sz);
		memset(&nextRoads[0], 0x3f, sizeof(int)*sz);
	}
};


#endif
