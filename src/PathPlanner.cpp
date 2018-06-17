#include "PathPlanner.h"

/*PATH PLANNER IMPLEMENTATION*/
/*----------------------------------------*/
PathPlanner::PathPlanner()
{
	current_state = new GoStraight();
}

void PathPlanner::Plan(vector<vector<double>> &sensor_fusion, vector<double> &ptx, vector<double> &pty, double &ref_vel)
{
	//get sensor data and previous points
  	this->sensor_data = sensor_fusion;
  	this->x_planned = ptx;
  	this->y_planned = pty;
  	this->vel_planned = ref_vel;

  	//calculate path according to current state
  	current_state->calculatePath(this);
  	
  	//modify the pointlists so that they can be passed to the simulator
  	ref_vel = this->vel_planned;
  	ptx = this->x_planned;
  	pty = this->y_planned; 
}

bool PathPlanner::isLeftLaneFree()
{
    if ((curr_lane - 1) < 0)
    {
    	return false;
    }
    
    return checkLane(curr_lane - 1);
}

bool PathPlanner::isRightLaneFree()
{
    if ((curr_lane + 1) > 2)
    {
    	return false;
    }
    
    return checkLane(curr_lane + 1);
}


/*
Checks if the lane denoted by integer lane is free i.e. there are no potential collisions.
*/
bool PathPlanner::checkLane(int lane)
{
	bool free = true;

	for(int i=0; i < sensor_data.size(); i++)
  	{
  		float d = sensor_data[i][6];

  		//if the other car is in same lane:
  		if(((d < (4+4*lane))) && (d >(4*lane)))
  		{
  			double vx = sensor_data[i][3];
  			double vy = sensor_data[i][4];
  			double check_speed = sqrt(vx*vx + vy*vy);
  			double check_car_s = sensor_data[i][5];

  			// when the function is used for checking the other lane,
  			// better to check if there exists a car currently close to ego vehicle.
  			if(fabs(check_car_s - car_s) < 10.0)
  			{
  				free = false;
  			}

  			check_car_s += (double)prev_horizon_size * .02 * check_speed;

  			//TODO: (future work)
  			// change hardcoded value "30" to a parametrized value depending on time and velocity
  			if((check_car_s > car_s) && ((check_car_s - car_s) < 30)) 
  			{
  				free = false;
  				break;
  			}	
  		}
  	}
  	return free;
}


/*
Code from QA video for creating waypoints from target frenet coordinates.
TODO: change hardcoded value to parametrized value.
*/
void PathPlanner::CreateWayPoints(int lane)
{
	//In frenet add evenly 30m spaced points ahead of the statring reference
  	//TODO instead of 30ms, make this proporional to speed
	//cout << "creating future points in frenet" << endl;
  	vector<vector<double>> next_wp;
  	for (int i=0; i<3;i++)
  	{
  		//cout << "calling getxy for next_wpoints" << endl;
  		next_wp.push_back(getXY(car_s + 30*(i+1), (2+4*lane), 
  			map_waypoints_s, map_waypoints_x, map_waypoints_y));
  		
  		//cout << "X" << i + 2 << " = " << next_wp[i][0] << endl;
  		//cout << "Y" << i + 2 << " = " << next_wp[i][1] << endl; 
  		x_planned.push_back(next_wp[i][0]);
  		y_planned.push_back(next_wp[i][1]);
  	}

  	for(int i=0; i<x_planned.size(); i++)
  	{
  		//transform into vehicle coordinate system
  		double shift_x = x_planned[i] - ref_x;
  		double shift_y = y_planned[i] - ref_y;

  		x_planned[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
  		y_planned[i] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));
  	}
}
/*---------END PATH PLANNER IMPL-------------------------------*/



/*STATES*/

/*GO STRAIGHT*/
/*--------------------------------------------------------------*/

/*
In state GoStraight, we check if there exists a possible collision in ego lane
If not, the speed is increased until the speed limit is reached
If there is a collision, first the left lane is checked, if possible; state is changed to lane change
If left lane is not available, right lane is checked, id possible; state is changed to lane change
If neither of the adjacent lanes are available, speed is decreased.
*/
void GoStraight::calculatePath(PathPlanner *veh)
{
	//cout << "In State GO STRAIGHT!!" << endl;
	bool too_close = false;

	too_close = !(veh->checkLane(veh->curr_lane));

  	if(too_close)
  	{
  		changeNextState(veh);
  	}
  	else if(veh->vel_planned < 49.5)
  	{
  		veh->vel_planned += 0.224;
  	}

  	
  	veh->CreateWayPoints(veh->curr_lane);	
}

void GoStraight::changeNextState(PathPlanner *veh)
{
	if(veh->isLeftLaneFree())
	{
		veh->SetState(new ChangeLane(veh->curr_lane, veh->curr_lane-1));
		delete this;
	}
	else if (veh->isRightLaneFree())
	{
		veh->SetState(new ChangeLane(veh->curr_lane, veh->curr_lane+1));
		delete this;
	}
	else
	{
		//cannot change lane, slow down
		veh->vel_planned -= 0.224;
	}
}
/*-----------------END GO STAIGHT-------------------------*/

/*-----------------CHANGE LANE----------------------------*/
ChangeLane::ChangeLane(int curr_lane, int target_lane)
{
	this->start_lane = curr_lane;
	this->target_lane = target_lane;
}

/*
In state change lane, state is preserved until the target lane is reached. 
After reaching the target lane, the state is switched to GoStraight.

One detail is if during the lane change a car appears in the target lane 
(i.e. a car which is changing lane or a car which has high relative speed 
which was not detected before)
The state is switched to goStraight, and staying in current lane is enforced.
*/
void ChangeLane::calculatePath(PathPlanner *veh)
{
	cout << "In State CHANGELANE!!" << endl;
	cout << "TARGET LANE : " << target_lane << endl;


	veh->CreateWayPoints(target_lane);

	if(!veh->checkLane(target_lane)) //check if a car appears, stay in lane.
	{		
		changeNextState(veh);
	}

    if(fabs(veh->car_d - (2.0+4.0*((double)target_lane))) < 1.0)
    {
    	changeNextState(veh);
    }
}

void ChangeLane::changeNextState(PathPlanner *veh)
{
	 	veh->SetState(new GoStraight());
 		delete this;
}
/*------------------END CHANGE LANE------------------------*/