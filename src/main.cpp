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

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


// Tuning parameter of the path planner

#define MAX_SPEED 48.5 //48 Speed limit with -2mph
#define TRACK_0 0 // Left lane
#define TRACK_1  1 // Center lane
#define TRACK_2 2  // Right lane
#define THRES_FRONT 24	//26 Threshold distance to objects in FRONT initializing maneuver
#define DIST_FRONT 20	//22 Safty distance to objects in FRONT required for safe maneuvers
#define DIST_REAR 14	//16 Safty distance to objects in REAR required for safe maneuvers
#define ACCEL 0.25 // Throttle/brake value 

// Further parameter in the code:
// a. Convert factor mph>m/s = 2.24
// b. Movement factor dt = 0.2ms
// c. Acceleration +/-0.25 which equals some +/-5m/s^2
// d. Path points spacing = 30m
// e. Lane width = 4m
// f. Max # path points in the planner = 50

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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// DONE:
// Evaluate if maneuver is safe
bool is_maneuver_safe(vector<vector<double>> sensor_fusion, int prev_path_size, double car_s, int target_lane, double ref_vel, double car_speed)
{
	bool maneuver_is_safe_flag = true;
//	bool no_queueing_flag = true; // flag if multiple obj in target lane queue behind
 
	// Loop through all detected objects 
	for (int j = 0; j < sensor_fusion.size(); j++) {
		float d = sensor_fusion[j][6]; // d position in frenet of detected car

		// Filter objects and find those in target lane, ignore rest
		if (d < (2 + 4 * target_lane + 2) && d > (2 + 4 * target_lane - 2)) {

			// Read model data of detected object 
			/*
			["sensor_fusion"] A 2d vector of cars and then that car's 
			[car's unique ID, car's x position in map coordinates, car's y position in map coordinates, 
			car's x velocity in m/s, car's y velocity in m/s, 
			car's s position in frenet coordinates, car's d position in frenet coordinates].
			*/
			int detected_obj_id = sensor_fusion[j][0];
			double vx = sensor_fusion[j][3];
			double vy = sensor_fusion[j][4];
			double detected_obj_speed = sqrt(vx * vx + vy * vy);
			double detected_obj_s = sensor_fusion[j][5];
			double detected_obj_d = sensor_fusion[j][6];

			// Predict obj's assumed s position (risk of obj's is also moving lateral)
			detected_obj_s += (double) prev_path_size * 0.02 * detected_obj_speed;
			// TODO: Predict obj's assumed D position and check if obj reduces lateral distance

			// Simulate if longitudinal distance margin in FRONT is insufficient 
			if ((detected_obj_s > car_s) && ((detected_obj_s - car_s) < DIST_FRONT))
			{
				// Simulate if slower obj's in front is reducing distance margin
				if (detected_obj_speed < ref_vel) { 
					maneuver_is_safe_flag = false; // invalidate maneuver, do not change lane
					cout << "obj ID=" << detected_obj_id << " detected - risky FRONT gap in target-lane #" << target_lane << endl;
				} else {
					// TODO: check if +/- acceleration (dv) of obj, if negative invalidate maneuver 
					cout << "obj ID=" << detected_obj_id << " detected - front gap is OPENING in target-lane #" << target_lane <<endl;
				}
			}

			// Simulate if longitudinal distance margin in REAR is insufficient, only check closest
			if ((detected_obj_s < car_s) && ((car_s - detected_obj_s) < DIST_REAR))
			{
				maneuver_is_safe_flag = false; // invalidate maneuver, do not change lane
				cout << "obj ID=" << detected_obj_id << " detected - risky REAR gap in target-lane #" << target_lane << endl;
				// TODO: check if obj is slower and not accelerating, if either one invalidate meneuver
				// TODO: handle change in distance margin if rear queueing (consider also order of sensor-fusion data)  
			}
		}
	}

	return maneuver_is_safe_flag;
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
	map_waypoints_x.push_back(x);
	map_waypoints_y.push_back(y);
	map_waypoints_s.push_back(s);
	map_waypoints_dx.push_back(d_x);
	map_waypoints_dy.push_back(d_y);
}

//DONE:
// Lane and speed of the simulator car at the start
int lane = TRACK_1; 		// The current lane, assume the car starts in lane 1 (center lane)
double ref_vel = 0.0;		// Start standstill, is the reference velocity of the car received from the simulator
double speed_limit = MAX_SPEED; // init adaptive speed behavior
cout << "Set TARGET SPEED = " << speed_limit << endl;

h.onMessage([&speed_limit,&ref_vel,&lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) 
{
// "42" at the start of the message means there's a websocket message event.
// The 4 signifies a websocket message
// The 2 signifies a websocket event
//auto sdata = string(data).substr(0, length);
//cout << sdata << endl;
if (length && length > 2 && data[0] == '4' && data[1] == '2') 
{

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

    // DONE:
    // The previous path being followed by the car
    int prev_path_size = previous_path_x.size();

    if (prev_path_size>0) {
      car_s = end_path_s;
    }

    // Reset controller status every cycle
    bool too_close = false;
    int go_left = lane - 1;
    int go_right = lane + 1;
    bool is_safe = false;
    int target_lane = lane;

    // LOOP through all sensored cars
    for(int i=0;i<sensor_fusion.size(); i++)
    {
      float d = sensor_fusion[i][6];

      // If obj is in CURRENT lane than simulate maneuver, else ignor
      if (d < (2 + 4 * lane + 2) &&
          d > (2 + 4 * lane - 2))
      {
        // READ speed and position
        double vx = sensor_fusion[i][3];                    // detected car's speed x direction
        double vy = sensor_fusion[i][4];                    // detected car's speed y direction
        double detected_obj_speed = sqrt(vx*vx + vy*vy);    // speed vector magnitude
        double detected_obj_s = sensor_fusion[i][5];        // s value of the detected car
        int detected_obj_id = sensor_fusion[i][0];          // id of the detected car

        // In 20ms from now PREDICT the future position of the detected obj  
        detected_obj_s += ((double)prev_path_size * 0.02 * detected_obj_speed);
        // TODO: Predict obj's assumed D position and check if obj reduces lateral distance

        // If obj position is predicted to be too close check potential meneuvers
        if((detected_obj_s > car_s) && (detected_obj_s - car_s) < THRES_FRONT)
        {
          too_close = true;
          cout << "obj ID=" << detected_obj_id << " detected - TOO CLOSE in current-lane #" << lane << endl;

          // Adapt SPEED limitation to obj speed but try to initiate takeover maneuver (lane change)
          speed_limit = detected_obj_speed * 2.24 + 1.0; // approach front objects with 1.0mph
          cout << "adapted to OBJ speed = " << speed_limit << endl;

          // If not in left most lane, simulate go LEFT checking if it's SAFE
          if (go_left >= TRACK_0) {
            is_safe = true;
            is_safe = is_maneuver_safe(sensor_fusion, prev_path_size, car_s, go_left, ref_vel, car_speed);
            if (is_safe)
              target_lane = go_left;
          }

          // If not in right most lane, simulate go RIGHT checking if it's SAFE
          if ((is_safe == false) && (go_right <= TRACK_2)) {
            is_safe = true;
            is_safe = is_maneuver_safe(sensor_fusion, prev_path_size, car_s, go_right, ref_vel, car_speed);
            if (is_safe)
              target_lane = go_right;
          }
        }
      } else if (!too_close) {
        speed_limit = MAX_SPEED; // back to max speed limit
      }
    } // end loop through detected obj's
      
      // Execute safe maneuver and speed control
      if (is_safe) {
        cout << "changed from LANE " << lane << " to " << target_lane << endl;
        lane = target_lane;
        if (target_lane != TRACK_1) {
          cout << "todo: check safty and change from lane " << target_lane << " to best lane [" << TRACK_1 << "]" << endl;
          // TODO: Always check and move back to center lane if possible (if best maneuver strategy=move to center lane)
          // OPTIONAL: best meneuver could also be predicted with cost function using entire sensor data
        // Set speed limit for target lane 
        speed_limit = MAX_SPEED;
        cout << "adapted to TARGET speed = " << speed_limit << endl << endl;
        }
      }
      else if (too_close)
      {
        // Slow down if front obj is too close and maneuver would be unsafe
        ref_vel -= ACCEL;
      }
      else if(ref_vel <= speed_limit)
      {
        // Adapt to speed limit set according to surrounding obj's 
        ref_vel += ACCEL;
      }
      // Collision detection and machine control completed

      // From here on code similar to Udacity project walk through
      // Begin to plan for new path
      
      // Path (x,y) waypoints, evenly spaced at 30m
      vector<double> ptsx;
      vector<double> ptsy;

      // Reference x,y,yaw states
      double ref_x = car_x;
      double ref_y = car_y;
      double ref_yaw = deg2rad(car_yaw);

      // If the previous path is almost empty set the current position for start
      if (prev_path_size < 2)
      {
        ptsx.push_back(car_x - cos(car_yaw)); 
        ptsx.push_back(car_x);  
        ptsy.push_back(car_y - sin(car_yaw));
        ptsy.push_back(car_y);
      }
      else
      {	// Else set the previous path's last two points for start
        ref_x = previous_path_x[prev_path_size-1];
        ref_y = previous_path_y[prev_path_size-1];

        double ref_x_prev = previous_path_x[prev_path_size-2];
        double ref_y_prev = previous_path_y[prev_path_size-2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // Set the two points making the path tangent to the previous path's two end points
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
      }

      // Add frenet points getting x and y coord transformed
      vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
      vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
      vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

      ptsx.push_back(next_wp0[0]);
      ptsx.push_back(next_wp1[0]);
      ptsx.push_back(next_wp2[0]);

      ptsy.push_back(next_wp0[1]);
      ptsy.push_back(next_wp1[1]);
      ptsy.push_back(next_wp2[1]);

      // ptsx and ptsy are set with the last two points and 50 evenly spaced points in global x.y coordinates

      // Transform the ptsx and ptsy vectors from global "x.y" to local "s.d" coordinates. 
      // Shift and rotate reference angle to zero degrees to make the math easier.
      for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;
        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
      } 
      // ptsx and ptsy are set with the last two points plus 50 evenly spaced points in local s.d coordinates

      // Build the next path to follow
      vector<double> next_x_vals;
      vector<double> next_y_vals;

      // Fit a spline and set its x and y points
      tk::spline s;
      s.set_points(ptsx, ptsy);

      // Start the new path with all the previous path points not used yet
      for(int i = 0;i<previous_path_x.size();i++)
      {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
      }

      // Calculate how to break up spline points to travel at the desired speed limit
      double target_x = 30;
      double target_y = s(target_x);  // use spline to get the y coordinate
      double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
      double x_add_on = 0;            // set to origin

      // Number of points of the path converted from miles/hour to m/s
      double N = (target_dist / (0.02 * ref_vel/2.24));

      // Add path planner points until max points are ready
      for(int i = 1;i <= 50 - previous_path_x.size(); i++){
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);   // use spline to get the y coordinate

        x_add_on = x_point;
        double x_ref = x_point;
        double y_ref = y_point;

        // Rotate and shift back to global coordinates
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw)) + ref_x;
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw)) + ref_y;

        // Set the points for the new path and continue
        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
      }
      // NEXT PATH TO FOLLOW HAS BEEN SET

      // Send the path of (x,y) points to the simulator so the robot car can follow them sequentially every 0.02 seconds
      json msgJson;
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
