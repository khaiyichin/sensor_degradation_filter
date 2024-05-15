#include "algorithms/CollectivePerception.hpp"

void CollectivePerception::ComputeLocalEstimate(const Params &params)
{
    double h = static_cast<double>(params.NumBlackTilesSeen);
    double t = static_cast<double>(params.NumObservations);

    if ((params.AssumedSensorAcc.at("b") == 1.0) && (params.AssumedSensorAcc.at("w") == 1.0)) // perfect sensor
    {
        local_vals_.X = h / t;
        local_vals_.Confidence = std::pow(t, 3) / (h * (t - h));
    }
    else // imperfect sensor
    {
        if (h <= (1.0 - params.AssumedSensorAcc.at("w")) * t)
        {
            double num = std::pow(params.AssumedSensorAcc.at("b") + params.AssumedSensorAcc.at("w") - 1.0, 2) * (t * std::pow(params.AssumedSensorAcc.at("w"), 2) - 2 * (t - h) * params.AssumedSensorAcc.at("w") + (t - h));
            double denom = std::pow(params.AssumedSensorAcc.at("w"), 2) * std::pow(params.AssumedSensorAcc.at("w") - 1.0, 2);

            local_vals_.X = 0.0;
            local_vals_.Confidence = num / denom;
        }
        else if (h >= params.AssumedSensorAcc.at("b") * t)
        {
            double num = std::pow(params.AssumedSensorAcc.at("b") + params.AssumedSensorAcc.at("w") - 1.0, 2) * (t * std::pow(params.AssumedSensorAcc.at("b"), 2) - 2 * h * params.AssumedSensorAcc.at("b") + h);
            double denom = std::pow(params.AssumedSensorAcc.at("b"), 2) * std::pow(params.AssumedSensorAcc.at("b") - 1.0, 2);

            local_vals_.X = 1.0;
            local_vals_.Confidence = num / denom;
        }
        else
        {
            local_vals_.X = (h / t + params.AssumedSensorAcc.at("w") - 1.0) / (params.AssumedSensorAcc.at("b") + params.AssumedSensorAcc.at("w") - 1);
            local_vals_.Confidence = (std::pow(t, 3) * std::pow(params.AssumedSensorAcc.at("b") + params.AssumedSensorAcc.at("w") - 1.0, 2)) / (h * (t - h));
        }
    }
}

void CollectivePerception::ComputeSocialEstimate(const std::vector<EstConfPair> &neighbor_vals)
{
    EstConfPair sum;

    auto lambda = [](const EstConfPair &left, const EstConfPair &right) -> EstConfPair
    {
        return EstConfPair(left.X + right.X * right.Confidence, left.Confidence + right.Confidence);
    };

    sum = std::accumulate(neighbor_vals.begin(), neighbor_vals.end(), EstConfPair(0.0, 0.0), lambda);

    // Assign the averages as social values
    social_vals_.X = (sum.Confidence == 0.0) ? 0.0 : sum.X / sum.Confidence;
    social_vals_.Confidence = sum.Confidence;
}

void CollectivePerception::ComputeInformedEstimate()
{
    if (local_vals_.Confidence == 0.0 && social_vals_.Confidence == 0.0)
    {
        informed_vals_.X = local_vals_.X;
    }
    else
    {
        informed_vals_.X = (local_vals_.Confidence * local_vals_.X + social_vals_.Confidence * social_vals_.X) / (local_vals_.Confidence + social_vals_.Confidence);
    }

    informed_vals_.Confidence = local_vals_.Confidence + social_vals_.Confidence;
}