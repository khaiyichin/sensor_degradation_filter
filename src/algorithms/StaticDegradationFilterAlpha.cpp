#include "algorithms/StaticDegradationFilterAlpha.hpp"

StaticDegradationFilterAlpha::StaticDegradationFilterAlpha(const std::shared_ptr<CollectivePerception::Params> &params_ptr,
                                                           const std::shared_ptr<CollectivePerception> &col_per_ptr)
{
    collective_perception_params_ptr_ = params_ptr;
    collective_perception_algo_ptr_ = col_per_ptr;
}

void StaticDegradationFilterAlpha::Estimate()
{
    double social_est = collective_perception_algo_ptr_->GetSocialVals().X;

    double numerator =
        (collective_perception_params_ptr_->NumBlackTilesSeen / collective_perception_params_ptr_->NumObservations) +
        social_est - 1.0;

    double denominator = 2 * social_est - 1;

    if (numerator > denominator)
    {
        estimate_["b"] = 1.0;
        estimate_["w"] = 1.0;
    }
    else if (numerator < 0.0)
    {
        estimate_["b"] = 0.0;
        estimate_["w"] = 0.0;
    }
    else
    {
        double updated_estimate = numerator / denominator;
        estimate_["b"] = updated_estimate;
        estimate_["w"] = updated_estimate;
    }
}