#ifndef PATH_PLN_
#define PATH_PLN_

#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "utils.h"

using namespace std;

class VehicleState;

/*
Holds a state machine using class VehicleState 
and plans the path according to vehicle state and sensor inputs
*/
class PathPlanner
{
public:
	PathPlanner();
	~PathPlanner(){delete this;}
	void SetState(VehicleState *state)
	{
		current_state = state;
	}

	void getPath();
	void setCarS(double s){ car_s = s;}
	void setCarLane(double d)
	{ 
		car_d = d;
		curr_lane = calculateLane(d);
	}

	void setCarState(double x, double y, double yaw)
	{
		ref_x = x; ref_y = y; ref_yaw = yaw;
	}

	void setPrevSize(int prev_size)
	{
		prev_horizon_size = prev_size;
	}

	void Plan(vector<vector<double>> &sensor_fusion, vector<double> &ptsx, vector<double> &ptsy, double &ref_vel);
    bool isLeftLaneFree();
    bool isRightLaneFree();
    bool checkLane(int lane);
    void CreateWayPoints(int lane);
	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

private:
	VehicleState* current_state;
	double car_s;
	double car_d;
	double ref_x;
	double ref_y;
	double ref_yaw;
	int curr_lane;
	int prev_horizon_size;

	vector<vector<double>> sensor_data;
	vector<double> x_planned;
	vector<double> y_planned;
	double vel_planned;

	int calculateLane(double d)
	{
		return (int)(d / 4);
	}

	friend class GoStraight;
	friend class ChangeLane;
};

/*
Abstract class for state machine implementation
*/
class VehicleState
{
public:
	VehicleState(){};
	virtual ~VehicleState(){}
	virtual void calculatePath(PathPlanner *veh){};
	virtual void changeNextState(PathPlanner *veh){};
};

/*
State for go straight
*/
class GoStraight : public VehicleState
{
public:
	GoStraight(){};
	~GoStraight(){};
	virtual void calculatePath(PathPlanner *veh);
	virtual void changeNextState(PathPlanner *veh);

};

/*
State for changing lanes. will be used for both changing right and to left
*/
class ChangeLane : public VehicleState
{
public:
	ChangeLane(){};
	ChangeLane(int curr_lane, int target_lane);
	~ChangeLane(){};
	virtual void calculatePath(PathPlanner *veh);
	virtual void changeNextState(PathPlanner *veh);
private:
	int start_lane;
	int target_lane; //target lane to change to
};


#endif