#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

int main() {
  uWS::Hub h;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  
  //start in lane 1 (lane 0 = left lane, lane 1 = middle lane, lane 2 = right lane
  int lane = 1;
  
  // set a reference velocity to target
  double ref_vel = 0;//mph

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          auto sensor_fusion = j[1]["sensor_fusion"]; 
          
            // set a constant maximum speed that we do not want the car to exceed
  			const double MAX_SPEED = 49.5; //mph  
  
  			const int LEFT_LANE = 0;
  			const int CENTER_LANE = 1;
  			const int RIGHT_LANE = 2;
     
          // last path that the car was following (usually 50)
          int prev_size = previous_path_x.size();
          
          // value to increment the velocity by when accelerating and decelerating
          double inc_vel = 0.4;//mph
          
          // Space to leave between our car and the car in front of us
          double safe_space = 30.0;
          double too_close = 15.0;
          
          // Space to leave between cars when deciding whether to change lanes
          double lane_change_space = 15.0;
          
          // If we have some prev points then we set the end of the previous path (s) to the start of our current path (s)
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          
          /*
           * Prediction - Where are there cars around us?
          */
         
          // declare booleans to track whether there are cars in the lanes around us
          bool car_ahead = false;
          bool car_ahead_close = false;
          bool car_left = false;
          bool car_right = false;
          
          // loop over all the cars in our sensor fusion vector
          // determine where the cars are around us
          for(int i = 0; i < sensor_fusion.size(); i++) {

            // lanes are 4m wide this calculation will tell us if the car is in our lane
            float d = sensor_fusion[i][6]; 
            
            // figure out what lane the other car is in currently
            int other_car_lane;
            if (d >= 0 && d < 4) {
              other_car_lane = LEFT_LANE;
            } else if (d >= 4 && d < 8) {
              other_car_lane = CENTER_LANE;
            } else if (d >= 8 && d < 12) {
              other_car_lane = RIGHT_LANE;
            } else { 
              continue;
            }
            
            //grab x, y, speed, and s location of the other car
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy); //helps to predict where car will be in the future
            double check_car_s = sensor_fusion[i][5]; //grab s value of car, how close it is to us

            // if we are using previous points this can project s value outward in time
            check_car_s += ((double)prev_size * .02 * check_speed);

            // if our car is in the same lane as the other car AND
            // the other car is in front of our car AND
            // the other car is within 30 meters
            if ((lane == other_car_lane) && (check_car_s > car_s) && ((check_car_s - car_s) < safe_space)) {
              car_ahead = true;
              
              if ((check_car_s - car_s) < too_close) {
                car_ahead_close = true;
              }
              // if the other car is in the lane to the right of us AND
              // the other car is within 60 meters of ours in the other lane
            } else if ((other_car_lane - lane == 1) 
                       && ((car_s - safe_space) < check_car_s) && ((car_s + safe_space) > check_car_s)) {
              car_right = true;
              
              // if the other car is in the lane to the left of us AND
              // the other car is within 60 meters of ours in the other lane
            } else if ((lane - other_car_lane == 1) 
                       && ((car_s - lane_change_space) < check_car_s) && ((car_s + safe_space) > check_car_s)) {
              car_left = true;
              }
            }

          // Execute lane changes or velocity changes depending on the situation around the car
          if(!car_ahead && (ref_vel < MAX_SPEED)) {
            ref_vel += inc_vel;
          } else if (car_ahead) {
            
            // LCL, if there is no car to the left and we are not in the left most lane
            if (!car_left && lane != LEFT_LANE) {
              lane--;
                
              // LCR if there is no car to the right and we are not in the right most lane
            } else if (!car_right && lane != RIGHT_LANE) {
              lane++;
              
              // decrease our speed if we are too close to the car and there is no available lane change
            } else if (!car_ahead_close) {
              ref_vel -= (inc_vel / 1.5);  
            } else {
              ref_vel -= (inc_vel * 1.5);
            }
          }
          
          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // later we will interpolate these points with a spline and fill it in with more points that control speed.
          vector<double> ptsx;
          vector<double> ptsy;
          
          // reference x, y , yaw states
          // either we will refernce the starting point as where the car is or at the previous paths and points.
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // if previous size is almost empty, use the car as a starting reference
          if (prev_size < 2) {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } 
          // use the previous path's end point as starting reference
          else {
            
            //Redefine reference state as previous path endpoint
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            // Look at previous 2 points and determine what angle the car was already going in
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
            
            //Use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // In Frenet add evenly 30m spaced points ahead of the starting reference
          // Vectors hold the position of the car in 30m, 60m, and 90m plus the previous two points
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
            
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for(int i=0;i<ptsx.size();i++)
          {
            // We are just doing the shift and rotate transformation that for origin angle is 0 degree.
            // Shift reference angle to 0 degrees
            // Means that if the car is facing at 45 degrees we shift the frame of reference so it seems like the car is facing straight ahead (0 degrees)
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0-ref_yaw)- shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw)+ shift_y * cos(0-ref_yaw));
            
          }
          
          // create a spline
          tk::spline s;
          
          // set points (x,y) points to the spline
          // these are the far spaced waypoints we are trying to get to.
          s.set_points(ptsx,ptsy);
          
          //Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // Start with all the previous path points from last time
          for(int i=0;i<previous_path_x.size();i++)
          {
            // Append points from previous path to the path planner
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate how to break up the spline point so that we travel at our desired velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist= sqrt(target_x*target_x + target_y*target_y);
          
          // This is where we are starting from along the x axis
          double x_add_on = 0.0;
          
          // fill up our rest of path planner after filling with the previous points, here we will always output the 50 points.
          // Previous path here will be a collection of the remaining points that the car did not actually get to during its last path
          for (int i=1;i<=50-previous_path_x.size();i++) {
            
            // Calculates distance between each point in the spline to keep us going near the target speed
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x/N);
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Convert back from local coordinates to the global coordinates
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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