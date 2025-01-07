#ifndef ORACLE_HPP
#define ORACLE_HPP

#include <unordered_map>
#include <memory>

#include "CollectivePerception.hpp"
#include "SensorDegradationFilter.hpp"

class Oracle : public SensorDegradationFilter
{
public:
    Oracle(const std::shared_ptr<CollectivePerception> &col_per_ptr, const std::unordered_map<std::string, double> &true_ground_sensor_acc);

    virtual void Estimate();

    std::shared_ptr<CollectivePerception> collective_perception_algo_ptr_;

private:
    const std::unordered_map<std::string, double> &tracked_actual_ground_sensor_acc_;
};

#endif