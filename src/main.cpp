#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}
double bx, by, cx, cy, dx, dy, xpred, ypred;
double tol = 0.01;
// Calculated center and radius by doing a least squares fit using target x and y points
// we could also iteratively calculate the center and update it doing a least squares fit.
// Using the first 3 points and adding the remaining as we get them.
double radius = 7.1663;
double circx = 0.1361;
double circy = 6.3295;
bool circ_found = true; bool reached_center = false;
int iteration = 0;
// Global variables, the distance and heading input to be given to the hunter car
double distance_diff, heading_diff;


// A helper function to obtain a heading and distance for the hunter to go to the center (circx, circy) given its current position
void go_to_center(const double& hunter_x, const double& hunter_y, double& hunter_heading, double& heading_diff, double& distance_diff) {
    double heading_to_point = atan2(circy - hunter_y, circx - hunter_x);

    if(abs(heading_to_point) > M_PI)
        heading_to_point = std::fmod(heading_to_point, 2*M_PI);
    
    heading_diff = heading_to_point - hunter_heading;
    
    if(abs(heading_diff) > M_PI)
        heading_diff = std::fmod(heading_diff, 2*M_PI);
    
    distance_diff = sqrt((circx - hunter_x)*(circx - hunter_x) + (circy - hunter_y)*(circy - hunter_y));
}
// Helper function to obtain heading and distance for hunter to go to any point (px, py)
void go_to_point(const double& px, const double& py, const double& hunter_x, const double& hunter_y, const double& hunter_heading, double& heading_diff, double& distance_diff) {
    double heading_to_point = atan2(py - hunter_y, px - hunter_x);
    while (heading_to_point > M_PI) heading_to_point-=2.*M_PI;
    while (heading_to_point <-M_PI) heading_to_point+=2.*M_PI;
    heading_diff = heading_to_point - hunter_heading;
    while (heading_diff > M_PI) heading_diff-=2.*M_PI;
    while (heading_diff<-M_PI) heading_diff+=2.*M_PI;
    distance_diff = sqrt((px - hunter_x)*(px - hunter_x) + (py - hunter_y)*(py - hunter_y));
}

// Helper function to calculate distance between points
double point_distance(const double& px, const double& py, const double& hunter_x, const double& hunter_y) {
    return sqrt((px - hunter_x)*(px - hunter_x) + (py - hunter_y)*(py - hunter_y));
}
int main(int argc, char* argv[])
{
  uWS::Hub h;

  // Create a UKF instance
  UKF ukf;
    ofstream outfile;
    if (argc > 1){
        outfile.open(argv[2],ofstream::out);
        outfile << "targetx" << ", ";
        outfile << "targety" << endl;
    }

  double target_x = 0.0;
  double target_y = 0.0;

  h.onMessage([&ukf,&target_x,&target_y, &outfile](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {
      	
      	
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          double hunter_x = std::stod(j[1]["hunter_x"].get<std::string>());
          double hunter_y = std::stod(j[1]["hunter_y"].get<std::string>());
          double hunter_heading = std::stod(j[1]["hunter_heading"].get<std::string>());
          
          string lidar_measurment = j[1]["lidar_measurement"];
          
          MeasurementPackage meas_package_L;
          istringstream iss_L(lidar_measurment);
    	  long long timestamp_L;

    	  // reads first element from the current line
    	  string sensor_type_L;
    	  iss_L >> sensor_type_L;

      	  // read measurements at this timestamp
      	  meas_package_L.sensor_type_ = MeasurementPackage::LASER;
          meas_package_L.raw_measurements_ = VectorXd(2);
          float px;
      	  float py;
          iss_L >> px;
          iss_L >> py;
          meas_package_L.raw_measurements_ << px, py;
          iss_L >> timestamp_L;
          meas_package_L.timestamp_ = timestamp_L;
            
    	  ukf.ProcessMeasurement(meas_package_L);
		 
    	  string radar_measurment = j[1]["radar_measurement"];
          
          MeasurementPackage meas_package_R;
          istringstream iss_R(radar_measurment);
    	  long long timestamp_R;

    	  // reads first element from the current line
    	  string sensor_type_R;
    	  iss_R >> sensor_type_R;

      	  // read measurements at this timestamp
      	  meas_package_R.sensor_type_ = MeasurementPackage::RADAR;
          meas_package_R.raw_measurements_ = VectorXd(3);
          float ro;
      	  float theta;
      	  float ro_dot;
          iss_R >> ro;
          iss_R >> theta;
          iss_R >> ro_dot;
          meas_package_R.raw_measurements_ << ro,theta, ro_dot;
          iss_R >> timestamp_R;
          meas_package_R.timestamp_ = timestamp_R;
          
    	  ukf.ProcessMeasurement(meas_package_R);

	  target_x = ukf.x_[0];
	  target_y = ukf.x_[1];
            if (outfile.is_open()){
                outfile << fixed << setprecision(4) << target_x << ", ";
                outfile << fixed << setprecision(4) << target_y << endl;            }
            iteration++;
            /* CODE TO CALCULATE CENTER OF CIRCLE BASED ON TARGET POSITIONS (any 3)
            int it_num = 100;
            
            int interval = 300;
            if(iteration == it_num){
                bx = target_x;
                by = target_y;
                cout << "bx = " << bx << endl;
                cout << "by = " << by << endl;
            }
            else if(iteration == it_num+interval) {
                cx = target_x;
                cy = target_y;
                cout << "cx = " << cx << endl;
                cout << "cy = " << cy << endl;
            }
            else if(iteration == it_num+interval*2) {
                dx = target_x;
                dy = target_y;
                cout << "dx = " << dx << endl;
                cout << "dy = " << dy << endl;
                double temp = cx*cx+cy*cy;
                double bc = (bx*bx + by*by - temp)/2.0;
                double cd = (temp - dx*dx - dy*dy)/2.0;
                double det = (bx-cx)*(cy-dy)-(cx-dx)*(by-cy);
                det = 1/det;
                circx = (bc*(cy-dy)-cd*(by-cy))*det;
                circy = ((bx-cx)*cd-(cx-dx)*bc)*det;

                cout << "circx = " << circx << endl;
                cout << "circy = " << circy << endl;
                radius = sqrt((cx-bx)*(cx-bx)+(cy-by)*(cy-by));
                cout << "radius = " << radius << endl;
                circ_found = true;
            }
            */
            
            double target_yaw_rate = ukf.x_[4];
            double target_velocity = ukf.x_[2];
            double target_heading = ukf.x_[3];
            double set_heading = atan2(target_y, target_x);
    	  double heading_to_target = atan2(target_y - hunter_y, target_x - hunter_x);
    	  while (heading_to_target > M_PI) heading_to_target-=2.*M_PI; 
    	  while (heading_to_target <-M_PI) heading_to_target+=2.*M_PI;
    	  //turn towards the target
    	  double heading_difference = heading_to_target - hunter_heading;
    	  while (heading_difference > M_PI) heading_difference-=2.*M_PI; 
    	  while (heading_difference <-M_PI) heading_difference+=2.*M_PI;

    	  double distance_difference = sqrt((target_y - hunter_y)*(target_y - hunter_y) + (target_x - hunter_x)*(target_x - hunter_x));
          json msgJson;
            
            // If we have not reached the center, then reached center will be false and we will head
            // to the center
            if(reached_center == false){
                cout << "Heading to center" << endl;
                //cout <<"Distance b/w hunter and center = " << point_distance(circx, circy, hunter_x, hunter_y) << endl;
                go_to_center(hunter_x, hunter_y, hunter_heading, heading_diff, distance_diff);
            }
            
            // If we are close to the center, we can assume we have reached it
            // If we are at the center, the advantage is that the time taken to go to any
            // point on the circle will be the same. So the time is fixed and known as the max velocity
            // of both the target and hunter are the same. We calculate the angular velocity based on the
            // velocity and radius. And predict the angle of the new point in the circle. Using this
            // we obtain the predicted x and y co-ordinate where we think the target car will be.
            if(point_distance(circx, circy, hunter_x, hunter_y) < 0.05 & reached_center == false) {
                long long time_to_point = radius / (target_velocity);
                double curr_angle = atan2(target_y - circy, target_x - circx);
                if(abs(curr_angle) > M_PI)
                    curr_angle = std::fmod(curr_angle, 2*M_PI);
                
                double omega = target_velocity / (radius) + 0.3;
                double pred_angle = curr_angle + omega*time_to_point;
                if(abs(pred_angle) > M_PI) {
                    pred_angle = std::fmod(pred_angle, 2*M_PI);
                }
                xpred = radius*cos(pred_angle) + circx;
                ypred = radius*sin(pred_angle) + circy;
                if(xpred < 0)
                    xpred -= 0.1;
                else
                    xpred += 0.1;
                if(ypred < 0)
                    ypred -= 0.1;
                else
                    ypred += 0.1;
                
                cout << "Reached center" << endl;
                cout << "Going to " << endl;
                cout << "xpred = " << xpred << endl;
                cout << "ypred = " << ypred << endl;
                reached_center = true;
            }
            // Finally we provide the command to the hunter to go this point.
            // Once we reached the center go to predicted point
            if(reached_center){
                cout << "Going to predicted point" << endl;
                go_to_point(xpred, ypred, hunter_x, hunter_y, hunter_heading, heading_diff, distance_diff);
                // If we have reached the predicted point and not caught the target car
                // Make reached_center false, return to the center and repeat the procedure!
                if(point_distance(xpred, ypred, hunter_x, hunter_y) < 0.05)
                    reached_center = false;
            }
            
             // Finally provide the command to the msg

            msgJson["turn"] = heading_diff;
            msgJson["dist"] = distance_diff;
            
           
            
            
          auto msg = "42[\"move_hunter\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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























































































