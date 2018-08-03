// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
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

int ClosestWaypoint(double x, double y,
                    const vector<double> &maps_x,
                    const vector<double> &maps_y)
{

	double closestLen   = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++) {

		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);

    if(dist < closestLen) {

			closestLen      = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta,
                 const vector<double> &maps_x,
                 const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle   = fabs(theta-heading);
  angle          = min(2*pi() - angle, angle);

  if(angle > pi()/4) {

    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {

      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y)
{

  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0){

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

	if(centerToPos <= centerToRef) {

		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++) {
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d,
                    const vector<double> &maps_s,
                    const vector<double> &maps_x,
                    const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
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


void load_Waypoints(vector<double> & map_waypoints_x,
                    vector<double> & map_waypoints_y,
                    vector<double> & map_waypoints_s,
                    vector<double> & map_waypoints_dx,
                    vector<double> & map_waypoints_dy)
{

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

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

}

struct Waypoints {

  vector<double> ptsx, ptsy;

  // Define the actual  (x,y) points we will use for the planner
  vector<double> next_x_vals, next_y_vals;

  //reference x,y, yaw estimates
  //either we will reference the starting point on where the car is
  //or at the previous paths and point
  double ref_x, ref_y, ref_yaw;

  //INPUT//
  double car_x, car_y, car_yaw, car_s;

  int prev_size, lane;

  vector<double> map_waypoints_s, map_waypoints_x, map_waypoints_y;
  vector<double> previous_path_x, previous_path_y;

  Waypoints (const int _prev_size, const int _lane,
             double _car_x, double _car_y, double _car_yaw, double _car_s,
             vector<double> _map_waypoints_s, vector<double> _map_waypoints_x,
             vector<double> _map_waypoints_y, vector<double> _previous_path_x,
             vector<double> _previous_path_y ):
    prev_size(_prev_size), lane(_lane),
    car_x(_car_x), car_y(_car_y), car_yaw(_car_yaw), car_s(_car_s),
    map_waypoints_s (_map_waypoints_s), map_waypoints_x (_map_waypoints_x),
    map_waypoints_y (_map_waypoints_y), previous_path_x (_previous_path_x),
    previous_path_y (_previous_path_y) {};

  void spaced_waypoints_generator ()
  {
    ref_x   = car_x;
    ref_y   = car_y;
    ref_yaw = deg2rad(car_yaw);
    //if previous size is almost empty, use the car as starting reference
    if (prev_size < 2) {

      //use two points that make the path tangent to the car
      double prev_car_x = car_x - cos(car_yaw);
      double prev_car_y = car_y - sin(car_yaw);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(car_x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(car_y);

    }
    // use the previous path's end point as starting reference
    else{

      //Redefine reference state as previous as staring reference

      ref_x  = previous_path_x[prev_size -1];
      ref_y  = previous_path_y[prev_size -1];

      double ref_x_prev = previous_path_x[prev_size -2];
      double ref_y_prev = previous_path_y[prev_size -2];

      ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);

    }

    // In Frenet add evenly 30m spaced points ahead of the starting reference
    std::vector<double> next_wp0 =
      getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    std::vector<double> next_wp1 =
      getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    std::vector<double> next_wp2 =
      getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
      ptsx.push_back(next_wp0[0]);

    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++){

      //shift car reference angle to 0 degrees
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;

      ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
      ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);

    }

  }

  void detailed_waypoints_generator (double ref_vel)
  {
    // create a spline
    tk::spline s;

    // set (x,y) points to the spline
    s.set_points(ptsx, ptsy);

    // Start with all of the previous path points from last time
    for (int i = 0; i < previous_path_x.size(); i++){

      next_x_vals.emplace_back(previous_path_x[i]);
      next_y_vals.emplace_back(previous_path_y[i]);
    }

    // Calculate how to break up spline points so that we travel
    // at our desired reference velocity
    double target_x    = 30.0;
    double target_y    = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

    double x_add_on   = 0;

    // Fill up the rest of our path planner after filling
    // it with previous points, here we will alwawys 50 set_points

    for (int i = 1; i <= 50 - previous_path_x.size(); i++){

      double N       = target_dist / (.02 * ref_vel/2.24);
      double x_point = x_add_on + (target_x) / N;
      double y_point = s(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      // rotate back to normal after rotating it earlier
      x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
      y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.emplace_back(x_point);
      next_y_vals.emplace_back(y_point);

    }

  }

};
