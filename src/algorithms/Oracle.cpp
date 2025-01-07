#include "algorithms/Oracle.hpp"

Oracle::Oracle(const std::shared_ptr<CollectivePerception> &col_per_ptr, const std::unordered_map<std::string, double> &actual_ground_sensor_acc)
    : SensorDegradationFilter(),
      tracked_actual_ground_sensor_acc_(actual_ground_sensor_acc)
{
    collective_perception_algo_ptr_ = col_per_ptr;
}

void Oracle::Estimate()
{
    params_ptr_->AssumedSensorAcc["b"] = tracked_actual_ground_sensor_acc_.at("b");
    params_ptr_->AssumedSensorAcc["w"] = tracked_actual_ground_sensor_acc_.at("w");
}