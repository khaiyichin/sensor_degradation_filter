#include "algorithms/CollectivePerception.hpp"

void CollectivePerception::ComputeLocalEstimate(const double &sensor_acc_b, const double &sensor_acc_w)
{
    double h = static_cast<double>(params_ptr_->NumBlackTilesSeen);
    double t = static_cast<double>(params_ptr_->NumObservations);

    if ((sensor_acc_b == 1.0) && (sensor_acc_w == 1.0)) // perfect sensor
    {
        local_vals_.X = h / t;
        local_vals_.Confidence = std::pow(t, 3) / (h * (t - h));
    }
    else // imperfect sensor
    {
        if (h <= (1.0 - sensor_acc_w) * t)
        {
            double num = std::pow(sensor_acc_b + sensor_acc_w - 1.0, 2) * (t * std::pow(sensor_acc_w, 2) - 2 * (t - h) * sensor_acc_w + (t - h));
            double denom = std::pow(sensor_acc_w, 2) * std::pow(sensor_acc_w - 1.0, 2);

            local_vals_.X = 0.0;
            local_vals_.Confidence = num / denom;
        }
        else if (h >= sensor_acc_b * t)
        {
            double num = std::pow(sensor_acc_b + sensor_acc_w - 1.0, 2) * (t * std::pow(sensor_acc_b, 2) - 2 * h * sensor_acc_b + h);
            double denom = std::pow(sensor_acc_b, 2) * std::pow(sensor_acc_b - 1.0, 2);

            local_vals_.X = 1.0;
            local_vals_.Confidence = num / denom;
        }
        else
        {
            local_vals_.X = (h / t + sensor_acc_w - 1.0) / (sensor_acc_b + sensor_acc_w - 1);
            local_vals_.Confidence = (std::pow(t, 3) * std::pow(sensor_acc_b + sensor_acc_w - 1.0, 2)) / (h * (t - h));
        }
    }

    // Modify confidence values to be non-zero to prevent numerical errors
    if (local_vals_.Confidence == 0.0)
    {
        local_vals_.Confidence = ZERO_APPROX;
    }
}

void CollectivePerception::ComputeSocialEstimate(const std::vector<EstConfPair> &neighbor_vals)
{
    EstConfPair sum;

    params_ptr_->MostRecentNeighborEstimates = neighbor_vals;

    auto lambda = [](const EstConfPair &left, const EstConfPair &right) -> EstConfPair
    {
        return EstConfPair(left.X + right.X * right.Confidence, left.Confidence + right.Confidence);
    };

    sum = std::accumulate(neighbor_vals.begin(), neighbor_vals.end(), EstConfPair(0.0, 0.0), lambda);

    // Assign the averages as social values
    social_vals_.X = sum.X / sum.Confidence;
    social_vals_.Confidence = sum.Confidence;
}

void CollectivePerception::ComputeInformedEstimate()
{
    informed_vals_.X = (local_vals_.Confidence * local_vals_.X + social_vals_.Confidence * social_vals_.X) / (local_vals_.Confidence + social_vals_.Confidence);
    informed_vals_.Confidence = local_vals_.Confidence + social_vals_.Confidence;
}