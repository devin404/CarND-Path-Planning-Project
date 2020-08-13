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
using std::atan2;

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

  //Starting point in lane 1
  int lane = 1;
  
  //Reference Velocity Initial referrance velocity with small tolerance to avoid crossing speed limit
  double ref_velocity = 0;  
  
  int count = 0; // To avoid left right changes in close instance a delay is introduce 100x20ms
    
  h.onMessage([&lane,&ref_velocity, &map_waypoints_x,&count, &map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int prev_path_size = previous_path_x.size();
          
          
          //Lane Change Logic and Deceleration Logic
         
          if(prev_path_size>0)
          {
            car_s = end_path_s;
          }
          
          bool obstacle_alert = false;
          bool car_on_left = false;
          bool car_on_right = false;
          //bool lane_change_left = false;
          //bool lane_change_right = false;

          
          //Iterate through sensor fusion data
          for(int i = 0; i< sensor_fusion.size(); i++)
          {
          	double d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double speed_obstacle = sqrt(vx*vx + vy*vy);
            double s_obstacle = sensor_fusion[i][5];
                
            //Extending obstacle time into the future
            
            
            if(d < (4*lane +4)  && d > (4*lane)) //Vehicle in the same lane
            {
              s_obstacle += prev_path_size*0.02*speed_obstacle;
              
              if((s_obstacle > car_s) && (s_obstacle - car_s)<20)
              {
                //Detects Obstacle in the lane
                obstacle_alert = true;                        
              } 
            }  
              
            //Preparing for lane change left or right
            if(obstacle_alert) 
            { 
               if(d < (4*(lane-1) +4)  && d > (4*(lane-1))) 
               {
                  if(abs(s_obstacle - (car_s+20)) < 30 && abs(s_obstacle - (car_s-20)) < 30)
                  {
                    car_on_left = true;
                  }

               }
               if(d < (4*(lane+1) +4)  && d > (4*(lane+1))) 
               {
                  if(abs(s_obstacle - (car_s+20)) < 30 && abs(s_obstacle - (car_s-20)) < 30)
                  {
                    car_on_right = true;
                  }
               }
             } //End Preparing for lane change left or right
                                  
           }
          
          //Acceleration euivalent to 1g =  9.8m/s2 0.4384
          if(obstacle_alert == true)
          {
            ref_velocity -= 0.4384; //Decelerate
            
            if(car_on_left == false)
            {                  
              if(lane!=0) //To prevent negative lane value
              { 
                count++;
               if(count == 10) //Delay for safe lane change
                {
                  lane = lane -1;
                 count =0;
                }
              }
              else
              {
                if(car_on_right == false)               
                {   count++;
                    if(count == 10)
                    { 
                      lane = lane +1;
                      count = 0;
                    }
                } 
              }
            }                    
          }
          else if(ref_velocity<49.5)
          {
            ref_velocity += 0.4384;
          }
               
         /*
          if(lane_change_left == true)
          {
            lane = lane -1;
            count = 0;
          }
          else if(lane_change_right == true)               
          {
            lane = lane +1;
            count = 0;
          } */
          
          //Anchor Points
          vector<double> pnts_x;
          vector<double> pnts_y;
          
          
          double x_ref = car_x;
          double y_ref = car_y;
          double x_ref_prev;
          double y_ref_prev;
          double yaw_ref = deg2rad(car_yaw);
          
          
          if(prev_path_size < 2) //Need minimum two points
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw); 
            pnts_x.push_back(prev_car_x);
            pnts_y.push_back(prev_car_y);
            pnts_x.push_back(car_x);
            pnts_y.push_back(car_y);                            
          }
          else
          {
            //Use the last two paths in the previous path to generate future path predictions.
            x_ref = previous_path_x[prev_path_size-1];
            y_ref = previous_path_y[prev_path_size-1];
            x_ref_prev = previous_path_x[prev_path_size-2];
            y_ref_prev = previous_path_y[prev_path_size-2];
            
            pnts_x.push_back(x_ref_prev);
            pnts_x.push_back(x_ref);
            pnts_y.push_back(y_ref_prev);
            pnts_y.push_back(y_ref); 
            
            
            //Calculation yaw based on the last two availanble points
            yaw_ref =  atan2(y_ref - y_ref_prev, x_ref - x_ref_prev);
                        
          }
          
                                                                                
          
          //Create 3 points in future in Frenet Cordinates at distance 30, 60 and 90 m                  
          vector<double>next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);  
          vector<double>next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);    
          vector<double>next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);                  
          pnts_x.push_back(next_wp0[0]);
          pnts_x.push_back(next_wp1[0]);
          pnts_x.push_back(next_wp2[0]);
                             
          pnts_y.push_back(next_wp0[1]);
          pnts_y.push_back(next_wp1[1]);
          pnts_y.push_back(next_wp2[1]);                   
                             
                             
          //Shift the Anchor points to Vehicle XY coordinate system                   
          double shift_x;
          double shift_y;                   
                             
          for(int i =0; i< pnts_x.size(); i++)
          {
            shift_x = pnts_x[i] - x_ref;
            shift_y = pnts_y[i] - y_ref;
            
            pnts_x[i] = (shift_x * cos(0 - yaw_ref) - shift_y * sin(0 - yaw_ref));
            pnts_y[i] = (shift_x * sin(0 - yaw_ref) + shift_y * cos(0 - yaw_ref));
          }
          
                         
          // Add previous paths into the path trajectory                
          for(int i = 0; i < prev_path_size; i++)
          {
             next_x_vals.push_back(previous_path_x[i]);
         	 next_y_vals.push_back(previous_path_y[i]);
          }
                         
          //Create a spline with the points set for path smoothning              
          tk::spline s;
		  s.set_points(pnts_x, pnts_y);                   
          
          /* Logic derived from Q and A Session
          To break the path into N segments approximated over the hypotenuse 
          */
          
          double target_x = 30;
          double target_y = s(30);
          double hypotenuse = sqrt(target_x*target_x + target_y*target_y);
          
          //NUM OF UNIFORM POINTS  each update 20ms -> 0.02, miles/hr to m/s conversion -> 2.24             
          double N = hypotenuse/(0.02*ref_velocity/2.24);                 
          
          double x_point = 0;  
          double y_point;  
          double x_add_on = 0;
                         
          // Our Trajectory size is 50, we already have points from previous path and need to fill the rest
          for(int i = 0; i < 50 - prev_path_size; i++)
          {
            x_point = x_add_on + target_x/N;
            y_point = s(x_point);
            double ref_x = x_point;
            double ref_y = y_point;
            
            x_add_on = x_point;
            
            //Transforming back to the XY cartesiam coordinates from Vehicle Coordinate Frame.
            x_point = (ref_x * cos(yaw_ref) - ref_y * sin(yaw_ref));
            y_point = (ref_x * sin(yaw_ref) + ref_y * cos(yaw_ref));
            
            x_point += x_ref;
            y_point += y_ref;
                       
            next_x_vals.push_back(x_point);
         	next_y_vals.push_back(y_point);           
               
          }          
          
          /* COMMENTED CODE              
          //Drive straight
          
          double dist_inc = 0.5;
          double next_s;
          double next_d;
          vector<double> XY;
          
          for (int i = 0; i < 50; ++i) {
            
             //Frenet Coordinate S and D to keep lane
             next_s = car_s + (i+1)*dist_inc;
             next_d = car_d; // Keep same lane as localisation data
             XY = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);   
         	 next_x_vals.push_back(XY[0]);
         	 next_y_vals.push_back(XY[1]);
          }

          //End Drive Straight
          
          */
          
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