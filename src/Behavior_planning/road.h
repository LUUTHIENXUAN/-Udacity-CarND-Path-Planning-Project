#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

using namespace std;

class Road {
public:

  int ego_key = -1;

  int num_lanes;

  vector<int> lane_speeds;

  int speed_limit;

  map<int, Vehicle> vehicles;

  int vehicles_added = 0;

  /**
  * Constructor
  */
  Road(int speed_limit, vector<int> lane_speeds);

  /**
  * Destructor
  */
  virtual ~Road();

  Vehicle get_ego();

  void ego_localization(double s);

  void populate_traffic();

  void advance();

  void add_ego(int lane_num, int s, double vel, vector<int> config_data);

  void add_vehicles_surrounding(vector<vector<double>> sensor_fusion, int prev_size);

  void behavior_planning();

};
