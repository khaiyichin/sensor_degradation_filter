#ifndef STATIC_DEGRADATION_FILTER_ALPHA_HPP
#define STATIC_DEGRADATION_FILTER_ALPHA_HPP

#include <unordered_map>
#include <memory>

#include "CollectivePerception.hpp"
#include "SensorDegradationFilter.hpp"

class StaticDegradationFilterAlpha : public SensorDegradationFilter
{
public:
    StaticDegradationFilterAlpha(const std::shared_ptr<CollectivePerception::Params> &params_ptr,
                                 const std::shared_ptr<CollectivePerception> &col_per_ptr);

    virtual void Estimate();

    std::shared_ptr<CollectivePerception::Params> collective_perception_params_ptr_;

    std::shared_ptr<CollectivePerception> collective_perception_algo_ptr_;

};

#endif