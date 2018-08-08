#include <functional>
#include <iterator>
#include <map>
#include <math.h>
#include "Behavior_planning/cost.h"
#include "Behavior_planning/vehicle.h"


const float REACH_GOAL     = 0.1 * pow(10, 5);
const float EFFICIENCY     = 2.0 * pow(10, 2);
const float MAX_ACCELERATE = 1.5 * pow(10, 6);
const float MAX_SPPED      = 1.5 * pow(10, 6);
const float LANE_CHANGE    = 1.5 * pow(10, 6);

/*
   Cost increases based on distance of intended lane
   (for planning a lane change) and final lane of trajectory.

   Cost of being out of goal lane also becomes larger as vehicle
   approaches goal distance.
*/
float goal_distance_cost(const Vehicle & vehicle,
                         const vector<Vehicle> & trajectory,
                         const map<int, vector<Vehicle>> & predictions,
                         map<string, float> & data)
{

    float cost;
    float distance = data["distance_to_goal"];

    if (distance > 0) {

      float delta_d
      = 2.0 * vehicle.goal_lane - data["intended_lane"] - data["final_lane"];

      cost  = 1 - 2*exp(-(abs(delta_d) / distance));

    } else cost = 1;


    return cost;
}

//make the vehicle drive in the fastest possible lane
/*
Cost becomes higher for trajectories with intended lane
and final lane that have traffic slower than vehicle's target speed.
*/
float inefficiency_cost(const Vehicle & vehicle,
                        const vector<Vehicle> & trajectory,
                        const map<int, vector<Vehicle>> & predictions,
                        map<string, float> & data)
{

    float proposed_speed_intended
     = vehicle_ahead_speed(predictions, data["intended_lane"], vehicle); //= lane_speed(predictions, data["intended_lane"]);

    // no vehicle
    if ( proposed_speed_intended < 0 ) proposed_speed_intended = vehicle.target_speed;

    float proposed_speed_final
    = vehicle_ahead_speed(predictions, data["final_lane"], vehicle); //= lane_speed(predictions, data["final_lane"]);

    // no vehicle
    if ( proposed_speed_final < 0) proposed_speed_final = vehicle.target_speed;

    float cost
    = (2.0 * vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;

    return cost;
}

float safety_lane_change_cost(const Vehicle & vehicle,
                              const vector<Vehicle> & trajectory,
                              const map<int, vector<Vehicle>> & predictions,
                              map<string, float> & data)
{

    if ( data["final_lane"] == data["intended_lane"] ) return 0.0;

    else {

      bool vehicle_beside
      = vehicle_beside_detection(predictions, data["intended_lane"], vehicle);

      if (vehicle_beside) return 1.0;
      else return 0.0;

    }
}

// Penalizes trajectories that exceed the speed limit.
float speed_limit_cost(const Vehicle & vehicle,
                       const vector<Vehicle> & trajectory,
                       const map<int, vector<Vehicle>> & predictions,
                       map<string, float> & data)
{

  //std::cout << " Speed: " << vehicle.v << " | "
  //                        << vehicle.max_speed <<std::endl;

  //if (vehicle.v < vehicle.max_speed) return 0.0;
  //else return 1.0;

  double BUFFER_V     = 0.05 * vehicle.max_speed;
  double STOP_COST    = 0.9;
  double target_speed = vehicle.max_speed - BUFFER_V;

  double cost;

  if (vehicle.v < target_speed)
     cost = STOP_COST * ((target_speed - vehicle.v)/ target_speed);

  else if (vehicle.v > vehicle.max_speed) cost = 1;

  else cost = (vehicle.v - target_speed)  / BUFFER_V;

  return cost;
}

// Penalizes trajectories that drive off the road.
float stays_off_road_cost(const Vehicle & vehicle,
                          const vector<Vehicle> & trajectory,
                          const map<int, vector<Vehicle>> & predictions,
                          map<string, float> & data)
{




}

// Penalizes trajectories that do not stay near the center of the lane.
float center_lane_cost(const Vehicle & vehicle,
                       const vector<Vehicle> & trajectory,
                       const map<int, vector<Vehicle>> & predictions,
                       map<string, float> & data)
{


}

// Penalizes trajectories that attempt to accelerate at a rate
// which is not possible for the vehicle.
float max_accelerate_cost(const Vehicle & vehicle,
                          const vector<Vehicle> & trajectory,
                          const map<int, vector<Vehicle>> & predictions,
                          map<string, float> & data)
{

  //std::cout << " Accelerate: " << vehicle.a << " | "
  //                             << vehicle.max_acceleration <<std::endl;

  if (vehicle.a < vehicle.max_acceleration) return 0.0;
  else return 1.0;

}

/*
All non ego vehicles in a lane have the same speed,
so to get the speed limit for a lane,
we can just find one vehicle in that lane.
*/
float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane)
{

    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
      int key         = it->first;
      Vehicle vehicle = it->second[0];

      if (vehicle.lane == lane && key != -1) return vehicle.v;

    }

    //Found no vehicle in the lane
    return -1.0;
}

float vehicle_ahead_speed(const map<int, vector<Vehicle>> & predictions,
                          int lane, const Vehicle & vehicle)
{

    int min_s          = vehicle.goal_s;
    bool found_vehicle = false;
    float speed        = 0;

    Vehicle temp_vehicle;

    for (map<int, vector<Vehicle>>::const_iterator it  = predictions.begin(); it != predictions.end(); ++it)
    {
        if (it->first == -1) continue;

        temp_vehicle = it->second[0];

        if (temp_vehicle.lane == lane && temp_vehicle.s > vehicle.s && temp_vehicle.s < min_s)
        {
          min_s         = temp_vehicle.s;
          speed         = temp_vehicle.v;
          found_vehicle = true;
        }
    }

    float shortest_dst = temp_vehicle.s - vehicle.s;

    if (found_vehicle && (shortest_dst < 40) ) return speed;

    else return -1.0; // no vehicle

}

bool vehicle_behind_detection(const map<int, vector<Vehicle>> & predictions,
                              int lane, const Vehicle & vehicle)
{

    int  max_s = -1;
    bool found_vehicle = false;

    Vehicle temp_vehicle;

    for (map<int, vector<Vehicle>>::const_iterator it  = predictions.begin(); it != predictions.end(); ++it)
    {
        if (it->first == -1) continue;

        temp_vehicle = it->second[0];

        if (temp_vehicle.lane == lane && temp_vehicle.s <= vehicle.s && temp_vehicle.s > max_s)
        {
            max_s         = temp_vehicle.s;
            // if this car is near to ego car
            if ((vehicle.s  - temp_vehicle.s) < 30)
            {
              found_vehicle = true;
              break;
            }

        }
    }

    return found_vehicle;
}

bool vehicle_beside_detection(const map<int, vector<Vehicle>> & predictions,
                              int lane, const Vehicle & vehicle)
{

    bool found_vehicle = false;

    Vehicle temp_vehicle;

    for (map<int, vector<Vehicle>>::const_iterator it  = predictions.begin(); it != predictions.end(); ++it)
    {
        if (it->first == -1) continue;

        temp_vehicle = it->second[0];

        if (temp_vehicle.lane == lane)
        {
            if (abs(vehicle.s  - temp_vehicle.s) < 20)
            {
              found_vehicle = true;
              break;
            }

        }
    }

    return found_vehicle;
}


/*
Sum weighted cost functions to get total cost for trajectory.
*/
float calculate_cost(const Vehicle & vehicle,
                     const map<int, vector<Vehicle>> & predictions,
                     const vector<Vehicle> & trajectory)
{

    map<string, float> trajectory_data
    = get_helper_data(vehicle, trajectory, predictions);

    float cost = 0.0;

    //Add additional cost functions here.
    vector<function<float(const Vehicle &, const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, float> &) >> cost_function_list
     = {goal_distance_cost, inefficiency_cost, max_accelerate_cost, speed_limit_cost, safety_lane_change_cost};

    vector<float> weight_list
     = {REACH_GOAL, EFFICIENCY, MAX_ACCELERATE, MAX_SPPED, LANE_CHANGE};

    vector<float> cost_list;
    for (int i = 0; i < cost_function_list.size(); i++) {

        float new_cost
        = weight_list[i] * cost_function_list[i](vehicle, trajectory, predictions, trajectory_data);

        //cost_list.emplace_back(new_cost);
        cost += new_cost;
    }

    //cost = cost_list[0] + cost_list[1] + cost_list[2] + cost_list[3]
    //       + (1 / cost_list[0]) * cost_list[1] * cost_list[1] ;

    return cost;
}

/*
Generate helper data to use in cost functions:
 indended_lane: the current lane +/- 1
                if vehicle is planning or executing a lane change.
 final_lane: the lane of the vehicle at the end of the trajectory.
 distance_to_goal: the distance of the vehicle to the goal.

Note that indended_lane and final_lane are both included to help
differentiate between planning and executing
a lane change in the cost functions.
*/
map<string, float> get_helper_data(const Vehicle & vehicle,
                                   const vector<Vehicle> & trajectory,
                                   const map<int, vector<Vehicle>> & predictions)
{

    map<string, float> trajectory_data;
    Vehicle trajectory_last = trajectory[1];

    float intended_lane;

    if (trajectory_last.state.compare("PLCL") == 0)
        intended_lane = trajectory_last.lane - 1;

    else if (trajectory_last.state.compare("PLCR") == 0)
        intended_lane = trajectory_last.lane + 1;

    else
     intended_lane = trajectory_last.lane;

    float distance_to_goal = vehicle.goal_s - trajectory_last.s;
    float final_lane       = trajectory_last.lane;

    trajectory_data["intended_lane"]    = intended_lane;
    trajectory_data["final_lane"]       = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;

    return trajectory_data;
}
