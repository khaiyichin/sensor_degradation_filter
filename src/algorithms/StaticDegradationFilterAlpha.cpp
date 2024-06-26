#include "algorithms/StaticDegradationFilterAlpha.hpp"

StaticDegradationFilterAlpha::StaticDegradationFilterAlpha(const std::shared_ptr<CollectivePerception> &col_per_ptr)
    : SensorDegradationFilter()
{
    collective_perception_algo_ptr_ = col_per_ptr;
}

void StaticDegradationFilterAlpha::Estimate()
{
    double social_est = collective_perception_algo_ptr_->GetSocialVals().X;

    double numerator =
        (static_cast<double>(collective_perception_algo_ptr_->GetParamsPtr()->NumBlackTilesSeen) /
         static_cast<double>(collective_perception_algo_ptr_->GetParamsPtr()->NumObservations)) +
        social_est - 1.0;

    double denominator = 2.0 * social_est - 1.0;

    if (numerator >= denominator)
    {
        params_ptr_->AssumedSensorAcc["b"] = 1.0 - ZERO_APPROX; // make it almost 1 to prevent numerical errors
        params_ptr_->AssumedSensorAcc["w"] = 1.0 - ZERO_APPROX;
    }
    else if (numerator <= 0.0)
    {
        params_ptr_->AssumedSensorAcc["b"] = ZERO_APPROX; // make it non-zero to prevent numerical errors
        params_ptr_->AssumedSensorAcc["w"] = ZERO_APPROX;
    }
    else
    {
        double updated_estimate = numerator / denominator;
        params_ptr_->AssumedSensorAcc["b"] = updated_estimate;
        params_ptr_->AssumedSensorAcc["w"] = updated_estimate;
    }
}