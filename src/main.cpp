#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper.h"

using namespace std;
using namespace utils;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}


int main()
{
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	int lane = params.lane;
	double ref_vel = params.ref_vel;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = MAX_S;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line))
	{
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
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	h.onMessage([&ref_vel, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
																															  uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;

		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{

			auto s = hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry")
				{
					// j[1] is the data JSON object
					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					//save the localisation data
					Car car{car_x, car_y, car_s, car_d, car_yaw, car_speed};
					lane = d2lane(car.d); //identify lane from d frenet coordinates
					
					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					const int prev_size = previous_path_x.size();
					if (prev_size > 0)
					{
						car_s = end_path_s;
					}

					//find the optimal lane using sensor fusion data
					std::pair<int, vector<double>> p1 = optimalLane( prev_size, car_s, car, sensor_fusion);
					int optimal_lane = p1.first;
					double front_gap = p1.second[0];
					double rear_gap = p1.second[1];
					cout << "optimal lane: " << optimal_lane << " front gap: " << front_gap << " rear gap: " << rear_gap << endl;

					//decide whether to change lane, slow down or speedup
					if ((optimal_lane == lane ) && tooCloseFront(front_gap)){//best to stay in lane and slow down
						cout << "Too close: Slowing down in lane" << endl;
						//if (gap > 0 && gap < 10)
						//	ref_vel = ref_vel/2.0;
						//else
						ref_vel -= params.vel_step;
					}
					else if(!tooClose(front_gap,rear_gap) && abs(optimal_lane -lane)==1) { //allow single lane change
						cout << "Too close: changing lane" << endl;
						lane = optimal_lane;
					}
					else if (ref_vel < params.speed_limit) //speed up if sufficient gap
					{
						cout << "Speeding up" << endl;
						ref_vel += params.vel_step;
					}
					json msgJson;

					// Way points in global coordinates
					vector<double> ptsx;
					vector<double> ptsy;

					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);

					//starting off - add 2 points with current s and 1 where we came from
					if (prev_size < 2)
					{
						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_y - sin(car_yaw);

						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);

						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);
					}
					//add 2 anchor points from previous path
					else
					{
						ref_x = previous_path_x[prev_size - 1];
						ref_y = previous_path_y[prev_size - 1];

						double prev_ref_x = previous_path_x[prev_size - 2];
						double prev_ref_y = previous_path_y[prev_size - 2];

						ref_yaw = atan2((ref_y - prev_ref_y), (ref_x - prev_ref_x));

						ptsx.push_back(prev_ref_x);
						ptsx.push_back(ref_x);

						ptsy.push_back(prev_ref_y);
						ptsy.push_back(ref_y);
					}
					//add anchor points at 30,60,90 m
					double d = lane2d(lane);
					vector<double> next_wp0 = getXY(car_s + 1*params.horizon_dist, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp1 = getXY(car_s + 2*params.horizon_dist, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp2 = getXY(car_s + 3*params.horizon_dist, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

					ptsx.push_back(next_wp0[0]);
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);

					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);

					//rotate to car's local coordinates
					for (int i = 0; i < ptsx.size(); i++)
					{
						double shift_x = ptsx[i] - ref_x;
						double shift_y = ptsy[i] - ref_y;

						ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
						ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
					}

					//project a spline using the anchor points
					tk::spline s;
					s.set_points(ptsx, ptsy);

					//push all the points from the previous path to the next path
					vector<double> next_x_vals;
					vector<double> next_y_vals;
					for (int i = 0; i < previous_path_x.size(); i++)
					{
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					/*add new waypoints to compensate for waypoints already consumed by the car 
					since the last update cycle*/
					double target_x = params.horizon_dist;
					double target_y = s(target_x);
					double target_dist = sqrt(target_x * target_x + target_y * target_y);

					double x_add_on = 0;
					//params.horizon - previous_path_x.size() represents new points size
					for (int i = 1; i <= params.horizon - previous_path_x.size(); i++)
					{
						//project new points onto spline 
						double N = (target_dist / (0.02 * ref_vel / 2.24));
						double x_point = x_add_on + (target_x) / N;
						double y_point = s(x_point);

						x_add_on = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						//rotate back to global after rotating it earlier
						x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
						y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

						x_point += ref_x;
						y_point += ref_y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
					}

					//send new path back to simulator
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
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
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
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
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
