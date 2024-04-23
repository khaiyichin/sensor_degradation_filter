#ifndef COLLECTIVE_PERCEPTION_ALGORITHM_HPP
#define COLLECTIVE_PERCEPTION_ALGORITHM_HPP

#include <string>
#include <vector>
#include <cmath>
#include <numeric>
#include <unordered_map>

class CollectivePerceptionAlgorithm
{
public:
    struct EstConfPair
    {
        /**
         * @brief Construct a new EstConfPair struct
         *
         */
        EstConfPair() : Id(""), X(0.0), Confidence(0.0) {}

        /**
         * @brief Construct a new EstConfPair struct
         *
         * @param x
         * @param confidence
         */
        EstConfPair(const double &x, const double &confidence) : Id(""), X(x), Confidence(confidence) {}

        inline EstConfPair operator/(const double &val)
        {
            return EstConfPair(X / val, Confidence / val);
        }

        friend inline bool operator==(const EstConfPair &lhs, const EstConfPair &rhs)
        {
            if (lhs.X == rhs.X && lhs.Confidence == rhs.Confidence)
            {
                return true;
            }

            else
            {
                return false;
            }
        }

        std::string Id; ///< Robot ID associated with the X and Confidence values

        double X; ///< Estimate value

        double Confidence; ///< Confidence value
    };

    struct Params
    {
        unsigned int NumBlackTilesSeen;
        unsigned int NumObservations;
        std::unordered_map<std::string, double> AssumedSensorAcc = {{"b", -1.0}, {"w", -1.0}};
    };

    CollectivePerceptionAlgorithm() {}

    void ComputeLocalEstimate(const Params &params);

    void ComputeSocialEstimate(const std::vector<EstConfPair> &neighbor_vals);

    void ComputeInformedEstimate();

    EstConfPair GetLocalVals() const { return LocalVals; }

    EstConfPair GetSocialVals() const { return SocialVals; }

    EstConfPair GetInformedVals() const { return InformedVals; }

private:
    EstConfPair LocalVals; ///< Local values

    EstConfPair SocialVals; ///< Social values

    EstConfPair InformedVals; ///< Informed values
};

#endif