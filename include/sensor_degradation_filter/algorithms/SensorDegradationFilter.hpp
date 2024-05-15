#ifndef SENSOR_DEGRADATION_FILTER_HPP
#define SENSOR_DEGRADATION_FILTER_HPP

#include <unordered_map>

class SensorDegradationFilter
{
public:
    SensorDegradationFilter();

    virtual ~SensorDegradationFilter() {}

    virtual void Estimate() = 0;

    std::unordered_map<std::string, double> GetAccuracyEstimates() const { return estimate_; };

protected:

    std::unordered_map<std::string, double> estimate_ = {{"b", -1.0}, {"w", -1.0}};
};

#endif