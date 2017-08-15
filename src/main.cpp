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
#include "spline.h" // import spline liabrary. 

using namespace std;


// constants
// high way lane id [0,1,2]; 
// factors for cost functions
int WEIGHT_EFFICIENCY = 1;
int WEIGHT_SAFETY = 100; 

// factors for collison cost calculation front and rear
float FACTOR_REF_FRONT = 1.0;
float FACTOR_REF_REAR = 0.6;
// mininum required safe distance between vehicles on highway
int DIST_REF = 30;
int DIST_REF_FRONT = 15;
int DIST_REF_REAR = 10;
// prediction waypoints number
int PATH_LENGTH = 50;
// lane width
int LANE_WIDTH = 4;
// target speed
float TARGET_SPEED = 49.5; // mph
// delta velocity
float DELTA_VEL = 0.33; // m/s
// detect vehicles with consideration of +/- LANE*LANE_MARGIN = +/-2.4m
float LANE_MARGIN = 0.6; 


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

// own functions 
// calculate minimum relative distance to the cloest front/rear car
// front: if calcuate the gap between front car, front is true. otherwise front is false.
float getMinGap(double s, int l, vector<vector<double>> sf, bool front)
{
  float minGap = 999;
  float minVel = 99;
  // default minGap 999 and minVel 99 will lead to 0 cost if ther is a empty lane.
  for(int i =0; i<sf.size(); i++)
  {
    float d = sf[i][6];
    if(d < (LANE_WIDTH*(0.5+l+LANE_MARGIN)) && d > (LANE_WIDTH*(0.5+l-LANE_MARGIN)))
    {
      double vx = sf[i][3];
      double vy = sf[i][4];		    
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sf[i][5];
      // predictions with constant speed and no lane change
      check_car_s += (PATH_LENGTH*0.02*check_speed);
      double gap = check_car_s - s;
      if(front) 
      {
        if(check_car_s >= s && gap <= minGap)
	{
 	  minGap = gap;
          minVel = check_speed;
	}
      }else 
      {
        if(check_car_s < s && abs(gap) <= minGap)
	{
          minGap = abs(gap);
          minVel = check_speed;
	}
      }
    }
  }
  return minGap/(50*0.02*minVel);   // realative to 1s check car displacement. return value is a relative value.
  //return minGap;
}

// efficient cost 
// factor=1.0
float s_diff_Cost(float ds)
{
  
  float cost =0;
  //float refd = DIST_REF_FRONT;
  float refd = FACTOR_REF_FRONT;

  if(ds <= 1*refd) 
  {
    cost = 1;
  }else if(ds > 3*refd)
  {
    cost = 0;
  }else
  {
    cost = (3*refd - ds)/(2*refd);
  }
  //std::cout << "ds" << ds << "  " << cost <<std::endl;
  return cost;
}

// safety cost
// factor=100
// cost function return value close to 100 means collison possibility, while close 200 means the distance is too small and lead to sudden brake.
float collisionCostFront(float ds)
{
  float cost = 0;
  //float refd = DIST_REF_FRONT;
  float refd = FACTOR_REF_FRONT;

  if(ds >= refd)
  {
    cost = 0;
  }else if(ds <= 0.5*refd)
  {
    cost = 2;
  }else if(ds < refd*0.7 && ds >= 0.5*refd)
  {
    cost = 1;
  }else
  {
    cost = (refd-ds)/(0.3*refd);
    //std::cout << "ds" << ds << "  " << cost <<std::endl;
  }
  return cost;
}

// safety cost
// factor=100
float collisionCostRear(float ds)
{
  float cost = 0;
  //float refd = DIST_REF_REAR;
  float refd = FACTOR_REF_REAR;

  if(ds >= refd)
  {
    cost = 0;
  }else if(ds < 0.5*refd)
  {
    cost = 1;
  }else
  {
    cost = (refd-ds)/(0.5*refd);
    //std::cout << "ds" << ds << "  " << cost <<std::endl;
  }
  return cost;
}

// calcuate cost of possible behaviors
// s: car end path s
// l: car lane 
// sf: car sensor fusion
// change: change lane or not, if keep lane change is false.
float calculateCost(double s, int l, vector<vector<double>> sf, bool change) 
{
  float cost = 0;

  float gap_front = getMinGap(s, l, sf, true);
  
  cost += s_diff_Cost(gap_front)*WEIGHT_EFFICIENCY;
  cost += collisionCostFront(gap_front)*WEIGHT_SAFETY; // front
  
  if(change)
  {
    float gap_rear = getMinGap(s, l, sf, false);
    cost += collisionCostRear(gap_rear)*WEIGHT_SAFETY; // rear
  }

  return cost;
}


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
  //string map_file_ = "../data/highway_map_bosch1.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  //double max_s = 5104.621;
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
  
  // start in lane 1;
  int lane = 1;
  // vehicle initial speed
  double vel = 0; //mph
  // int cycle number
  int i_cycle = 0;

  h.onMessage([&vel,&lane,&i_cycle,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
		// car reference position 
		double pos_x = car_x;
          	double pos_y = car_y;
          	double angle = deg2rad(car_yaw);

          	int prev_size = previous_path_x.size();		
		
		// list of waypoints for spline in x,y cs
                vector<double> ptsx;
		vector<double> ptsy;

                // use ego vehicle prediction end_path_s to compare with close cars 
		if(prev_size > 0)
		{
		  car_s = end_path_s;
		}
	
		// flag to KL
		bool KL = true;
		// flag to LCL
		bool LCL = false;
		// flag to LCR
		bool LCR = false;
	        // cost variables for possible behaviors
		float cost_KL = 999;
                float cost_LCL = 999;
                float cost_LCR = 999;
		float cost_LCLL = 999;
		float cost_LCRR = 999;
		// flag to increase speed or not
  		bool flag_reduce_vel = false;
  		// flag to increase acceleration or not
		bool flag_increase_acc = false;
                bool flag_reduce_acc = false;

		// detect front car  
		for(int i =0; i<sensor_fusion.size(); i++)
		{
		  float d = sensor_fusion[i][6];
		  if(d < (LANE_WIDTH*(0.5+lane+LANE_MARGIN)) && d > (LANE_WIDTH*(0.5+lane-LANE_MARGIN)))
		  {
		    double vx = sensor_fusion[i][3];
    		    double vy = sensor_fusion[i][4];		    
		    double check_speed = sqrt(vx*vx+vy*vy);
		    double check_car_s = sensor_fusion[i][5];

   		    check_car_s += ((double)prev_size*0.02*check_speed);

		    if((check_car_s > car_s) &&((check_car_s-car_s) <= DIST_REF))
		    {
		      // behavior_planner model to determine
		      // LCL / LCR / KL
  		      // +/- velocity
                      // +/- acceleration
		      // calcuate cost of possible behaviors according to current lane
		      if(lane == 0)
		      {
                        cost_LCL = 999;
			cost_KL = calculateCost(car_s, lane, sensor_fusion, false);
			cost_LCR = calculateCost(car_s, (lane+1), sensor_fusion, true);
			cost_LCRR = calculateCost(car_s, (lane+2), sensor_fusion, true);
		      }else if(lane == 1)
		      {
		        cost_LCL = calculateCost(car_s, (lane-1), sensor_fusion, true);
		        cost_KL  = calculateCost(car_s, lane, sensor_fusion, false);
		        cost_LCR = calculateCost(car_s, (lane+1), sensor_fusion, true);
		      }else if(lane == 2)
		      {
			cost_LCL = calculateCost(car_s, (lane-2), sensor_fusion, true);
			cost_LCL = calculateCost(car_s, (lane-1), sensor_fusion, true);
		        cost_KL  = calculateCost(car_s, lane, sensor_fusion, false);
		        cost_LCR = 999;
		      }
		      
		      if(cost_LCLL < cost_LCL && cost_LCL < .3*WEIGHT_SAFETY)
  		      {
 			cost_LCL = cost_LCLL;
                      }

	 	      if(cost_LCRR < cost_LCR && cost_LCR < .3*WEIGHT_SAFETY)
  		      {
 			cost_LCR = cost_LCRR;
                      }

		      // determine best behavior according to the min cost
                      if(cost_KL <= cost_LCL && cost_KL <= cost_LCR)
		      {
			KL = true;
			LCL = false;
			LCR = false;
			flag_reduce_vel = true;
                        if(cost_KL >1.5*WEIGHT_SAFETY)
                        {
                          flag_reduce_acc = true;
                        }
		      }else if(cost_LCL <= cost_KL && cost_LCL <= cost_LCR && cost_LCL < WEIGHT_SAFETY) 
		      {
		        KL = false;
  			LCL  = true;
			LCR  = false;
                        if(vel < TARGET_SPEED*0.8)
  	 		{
			  flag_increase_acc = true;
			}
			if(cost_LCL >1.5*WEIGHT_SAFETY)
                        {
                          flag_reduce_acc = true;
                        }
 		      }else if(cost_LCR <= cost_KL && cost_LCR <= cost_LCL && cost_LCR < WEIGHT_SAFETY)
		      {
			KL = false;
  			LCL  = false;
		        LCR = true;
			if(vel < TARGET_SPEED*0.8)
  	 		{
			  flag_increase_acc = true;
			}
                        if(cost_LCR >1.5*WEIGHT_SAFETY)
                        {
                          flag_reduce_acc = true;
                        }
		      }
		    }
		  }
		}
		
		// KL or LCL or LCR
                // i_cycle % 4 = 0 to avoid continuous lane change and high acceleration.
		if(LCL && (i_cycle % 4 == 0))
		{
		  lane -=1;
		  std::cout << "lane change left: cost of LCL, KL, LCR:" << cost_LCL << "  "<< cost_KL << "  " << cost_LCR << std::endl;
		}else if(LCR && (i_cycle % 4 == 0))
		{
		  lane +=1;
		  std::cout << "lane change right: cost of LCL, KL, LCR:" << cost_LCL << "  "<< cost_KL << "  " << cost_LCR << std::endl;
		}else 
		{
		  //std::cout << "keep lane: cost of LCL, KL, LCR:" << cost_LCL << "  "<< cost_KL << "  " << cost_LCR << std::endl;
		}

		// increase velocity or decrease velocity
		if(flag_reduce_vel)
		{
	 	  vel -= DELTA_VEL;
		}
		else if(vel < TARGET_SPEED)
		{
		  vel += DELTA_VEL;
		}

		// increase a or decrease a
		if(flag_increase_acc or vel < 0.5*TARGET_SPEED)
                {
                  vel += DELTA_VEL;
                }		
		if(flag_reduce_acc)
                {
                  vel -= DELTA_VEL*2;
                }
                i_cycle += 1;
                
		// trajectory generation model 
          	if(prev_size < 2)
          	{
              	  // determine previous point according to car_yaw
		  double prev_pos_x = car_x - cos(car_yaw);
    		  double prev_pos_y = car_y - sin(car_yaw);

		  ptsx.push_back(prev_pos_x);
		  ptsx.push_back(car_x);

		  ptsy.push_back(prev_pos_y);
		  ptsy.push_back(car_y);
          	}
          	else
          	{
              	  pos_x = previous_path_x[prev_size-1];
              	  pos_y = previous_path_y[prev_size-1];

              	  double prev_pos_x = previous_path_x[prev_size-2];
              	  double prev_pos_y = previous_path_y[prev_size-2];
              	  angle = atan2(pos_y-prev_pos_y,pos_x-prev_pos_x);

    		  ptsx.push_back(prev_pos_x);
		  ptsx.push_back(pos_x);

		  ptsy.push_back(prev_pos_y);
		  ptsy.push_back(pos_y);
          	}

		
		vector<double> next_wp0 = getXY(car_s+1*DIST_REF, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp1 = getXY(car_s+2*DIST_REF, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp2 = getXY(car_s+3*DIST_REF, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

 		ptsx.push_back(next_wp0[0]);
		ptsx.push_back(next_wp1[0]);
		ptsx.push_back(next_wp2[0]);

		ptsy.push_back(next_wp0[1]);
		ptsy.push_back(next_wp1[1]);
		ptsy.push_back(next_wp2[1]);

		for(int i = 0; i < ptsx.size(); i++)
		{
   		  // transform to car local cs
		  double shift_x = ptsx[i] - pos_x;
		  double shift_y = ptsy[i] - pos_y;

		  ptsx[i] = (shift_x*cos(0-angle)-shift_y*sin(0-angle));
   		  ptsy[i] = (shift_x*sin(0-angle)+shift_y*cos(0-angle));
		}

		// create a spline
   		tk::spline s;
		s.set_points(ptsx,ptsy);

		// carry over previous path waypoints from previous_path_x and previous_path_y		
          	for(int i = 0; i < prev_size; i++)
          	{
              	  next_x_vals.push_back(previous_path_x[i]);
              	  next_y_vals.push_back(previous_path_y[i]);
          	}
		
		// calculate how to break up spline points so that we can travel at our designed reference velocity
    		double target_x = 30.0;
		double target_y = s(target_x);
		double target_dist = sqrt(target_x*target_x+target_y*target_y);
		double x_add_on = 0;

    		for(int i = 0; i < 50 - previous_path_x.size(); i++)
    		{
                  //		
                  double N = (target_dist/(0.02*vel/2.24));
		  double x_point = x_add_on + target_x/N;
		  double y_point = s(x_point);

		  x_add_on = x_point;
			
		  double x_ref = x_point;
		  double y_ref = y_point;
		  // rotate back to normal after rotating it earlier
                  x_point = (x_ref*cos(angle)-y_ref*sin(angle));
		  y_point = (x_ref*sin(angle)+y_ref*cos(angle));
		  x_point += pos_x;
		  y_point += pos_y;

          	  next_x_vals.push_back(x_point);
          	  next_y_vals.push_back(y_point);
    		}
		
                // END


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
















































































