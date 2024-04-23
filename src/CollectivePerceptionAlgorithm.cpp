#include "CollectivePerceptionAlgorithm.hpp"

void CollectivePerceptionAlgorithm::ComputeLocalEstimate(const Params &params)
{
    double h = static_cast<double>(params.NumBlackTilesSeen);
    double t = static_cast<double>(params.NumObservations);

    if ((params.AssumedSensorAcc.at("b") == 1.0) && (params.AssumedSensorAcc.at("w") == 1.0)) // perfect sensor
    {
        LocalVals.X = h / t;
        LocalVals.Confidence = std::pow(t, 3) / (h * (t - h));
    }
    else // imperfect sensor
    {
        if (h <= (1.0 - params.AssumedSensorAcc.at("w")) * t)
        {
            float num = std::pow(params.AssumedSensorAcc.at("b") + params.AssumedSensorAcc.at("w") - 1.0, 2) * (t * std::pow(params.AssumedSensorAcc.at("w"), 2) - 2 * (t - h) * params.AssumedSensorAcc.at("w") + (t - h));
            float denom = std::pow(params.AssumedSensorAcc.at("w"), 2) * std::pow(params.AssumedSensorAcc.at("w") - 1.0, 2);

            LocalVals.X = 0.0;
            LocalVals.Confidence = num / denom;
        }
        else if (h >= params.AssumedSensorAcc.at("b") * t)
        {
            float num = std::pow(params.AssumedSensorAcc.at("b") + params.AssumedSensorAcc.at("w") - 1.0, 2) * (t * std::pow(params.AssumedSensorAcc.at("b"), 2) - 2 * h * params.AssumedSensorAcc.at("b") + h);
            float denom = std::pow(params.AssumedSensorAcc.at("b"), 2) * std::pow(params.AssumedSensorAcc.at("b") - 1.0, 2);

            LocalVals.X = 1.0;
            LocalVals.Confidence = num / denom;
        }
        else
        {
            LocalVals.X = (h / t + params.AssumedSensorAcc.at("w") - 1.0) / (params.AssumedSensorAcc.at("b") + params.AssumedSensorAcc.at("w") - 1);
            LocalVals.Confidence = (std::pow(t, 3) * std::pow(params.AssumedSensorAcc.at("b") + params.AssumedSensorAcc.at("w") - 1.0, 2)) / (h * (t - h));
        }
    }
}

void CollectivePerceptionAlgorithm::ComputeSocialEstimate(const std::vector<EstConfPair> &neighbor_vals)
{
    EstConfPair sum;

    auto lambda = [](const EstConfPair &left, const EstConfPair &right) -> EstConfPair
    {
        return EstConfPair(left.X + right.X * right.Confidence, left.Confidence + right.Confidence);
    };

    sum = std::accumulate(neighbor_vals.begin(), neighbor_vals.end(), EstConfPair(0.0, 0.0), lambda);

    // Assign the averages as social values
    SocialVals.X = (sum.Confidence == 0.0) ? 0.0 : sum.X / sum.Confidence;
    SocialVals.Confidence = sum.Confidence;
}

void CollectivePerceptionAlgorithm::ComputeInformedEstimate()
{
    if (LocalVals.Confidence == 0.0 && SocialVals.Confidence == 0.0)
    {
        InformedVals.X = LocalVals.X;
    }
    else
    {
        InformedVals.X = (LocalVals.Confidence * LocalVals.X + SocialVals.Confidence * SocialVals.X) / (LocalVals.Confidence + SocialVals.Confidence);
    }

    InformedVals.Confidence = LocalVals.Confidence + SocialVals.Confidence;
}