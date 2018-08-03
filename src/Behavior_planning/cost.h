#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

float calculate_cost(const Vehicle & vehicle,
                     const map<int,
                     vector<Vehicle>> & predictions,
                     const vector<Vehicle> & trajectory);

float goal_distance_cost(const Vehicle & vehicle,
                         const vector<Vehicle> & trajectory,
                         const map<int, vector<Vehicle>> & predictions,
                         map<string, float> & data);

float inefficiency_cost(const Vehicle & vehicle,
                        const vector<Vehicle> & trajectory,
                        const map<int, vector<Vehicle>> & predictions,
                        map<string, float> & data);

float speed_limit_cost(const Vehicle & vehicle,
                       const vector<Vehicle> & trajectory,
                       const map<int, vector<Vehicle>> & predictions,
                       map<string, float> & data);

float stays_off_road_cost(const Vehicle & vehicle,
                          const vector<Vehicle> & trajectory,
                          const map<int, vector<Vehicle>> & predictions,
                          map<string, float> & data);

float center_lane_cost(const Vehicle & vehicle,
                       const vector<Vehicle> & trajectory,
                       const map<int, vector<Vehicle>> & predictions,
                       map<string, float> & data);

float max_accelerate_cost(const Vehicle & vehicle,
                          const vector<Vehicle> & trajectory,
                          const map<int, vector<Vehicle>> & predictions,
                          map<string, float> & data);

float lane_speed(const map<int, vector<Vehicle>> & predictions,
                 int lane);

bool vehicle_ahead_detection(const map<int, vector<Vehicle>> & predictions,
                             int lane, const Vehicle & vehicle);

map<string, float> get_helper_data(const Vehicle & vehicle,
                                   const vector<Vehicle> & trajectory,
                                   const map<int, vector<Vehicle>> & predictions);

#endif
