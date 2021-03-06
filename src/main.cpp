#include <fstream>

#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "PathPlanner.h"
#include "utils.h"


// for convenience
using json = nlohmann::json;



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;
  

  //PathPlanner - will use FSM in order to take appropriate action
  PathPlanner pp;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	pp.map_waypoints_x.push_back(x);
  	pp.map_waypoints_y.push_back(y);
  	pp.map_waypoints_s.push_back(s);
  	pp.map_waypoints_dx.push_back(d_x);
  	pp.map_waypoints_dy.push_back(d_y);
  }

  double ref_vel = 0.0;
  h.onMessage([&pp, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	//auto sensor_fusion = j[1]["sensor_fusion"];
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	int prev_size = previous_path_x.size();
          	bool too_close = false;
          	int lane = calculateLane(car_d);
          	
          	cout << "car_d = " << car_d << " lane = " << lane << endl;
          	cout << "previous path size " << prev_size << endl;

          	if(prev_size > 0)
          	{
          		car_s = end_path_s;
          	}

          	//set the current values for the planner
          	pp.setCarS(car_s);
          	pp.setCarLane(car_d);
          	pp.setPrevSize(prev_size);

          	vector<double> ptsx;
          	vector<double> ptsy;

          	//reference x, y, yaw
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);
          	

          	if(prev_size <= 2)
          	{
          		//use two points that make tangent to the car
          		double prev_car_x = car_x - cos(ref_yaw);
          		double prev_car_y = car_y - sin(ref_yaw);

          		ptsx.push_back(prev_car_x);
          		ptsx.push_back(ref_x);

          		ptsy.push_back(prev_car_y);
          		ptsy.push_back(ref_y);
          	}
          	else
          	{
          		//cout << "setting previous points to anchor" << endl;
          		ref_x = previous_path_x[prev_size - 1];
          		ref_y = previous_path_y[prev_size - 1];

          		double ref_x_prev = previous_path_x[prev_size - 2];
          		double ref_y_prev = previous_path_y[prev_size - 2];

          		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

          		//use last two points from the previous path
          		ptsx.push_back(ref_x_prev);
          		ptsx.push_back(ref_x);

          		ptsy.push_back(ref_y_prev);
          		ptsy.push_back(ref_y);
          	}

          	//set vehicle state in xy coordinates for planner and perform planning        	
			pp.setCarState(ref_x, ref_y, ref_yaw);
          	pp.Plan(sensor_fusion, ptsx, ptsy, ref_vel);
          	
          	// Creating the spline and adding the points to next_vals, following the QA code:
          	//create a spline
          	tk::spline s;

          	cout << "creating spline" << endl;
          	//set x,y points to the spline
          	s.set_points(ptsx, ptsy);

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// start with all of the previous points from last time
          	for(int i = 0; i<prev_size; i++)
          	{
          		next_x_vals.push_back(previous_path_x[i]);
          		next_y_vals.push_back(previous_path_y[i]);
          	}

          	//inacurrate calculation from QA video:
          	//-------------------------------------------------------------------
			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist = sqrt(target_x * target_x + target_y * target_y);

			double x_add_on = 0;
			//Fill up remaining of the planned path
			
			for(int i=0; i<= 50 - prev_size; i++)
			{
				double N = target_dist / (.02 * ref_vel / 2.24); //TODO move the unit conversion to upper parts
				double x_point = x_add_on + target_x / N;
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				//transform back to world coor
				x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
				y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			}
			//-------------------------------------------------------------------


          	// define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
