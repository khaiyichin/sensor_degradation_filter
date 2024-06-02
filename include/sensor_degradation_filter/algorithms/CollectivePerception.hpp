#ifndef COLLECTIVE_PERCEPTION_HPP
#define COLLECTIVE_PERCEPTION_HPP

#include <string>
#include <vector>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <memory>

class CollectivePerception
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

        inline EstConfPair operator+(const EstConfPair &obj)
        {
            return EstConfPair(this->X + obj.X, this->Confidence + obj.Confidence);
        }

        inline EstConfPair operator-(const EstConfPair &obj)
        {
            return EstConfPair(this->X - obj.X, this->Confidence - obj.Confidence);
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
        void Reset()
        {
            NumBlackTilesSeen = 0;
            NumObservations = 0;
        }

        unsigned int NumBlackTilesSeen = 0;

        unsigned int NumObservations = 0;

        std::vector<EstConfPair> MostRecentNeighborEstimates;
    };

    CollectivePerception() : params_ptr_(std::make_shared<Params>()) {}

    void Reset()
    {
        local_vals_ = EstConfPair(0.5, 0.0);
        social_vals_ = EstConfPair();
        informed_vals_ = EstConfPair();
        params_ptr_->Reset();
    }

    void ComputeLocalEstimate(const double &sensor_acc_b, const double &sensor_acc_w);

    void ComputeSocialEstimate(const std::vector<EstConfPair> &neighbor_vals);

    void ComputeInformedEstimate();

    EstConfPair GetLocalVals() const { return local_vals_; }

    EstConfPair GetSocialVals() const { return social_vals_; }

    EstConfPair GetInformedVals() const { return informed_vals_; }

    std::shared_ptr<Params> GetParamsPtr() const { return params_ptr_; }

private:
    EstConfPair local_vals_ = EstConfPair(0.5, 0.0); ///< Local values

    EstConfPair social_vals_ = EstConfPair(); ///< Social values

    EstConfPair informed_vals_ = EstConfPair(); ///< Informed values

    std::shared_ptr<Params> params_ptr_; ///< Pointer to parameters struct
};

#endif