#ifndef DYNAMIC_DEGRADATION_FILTER_DELTA_HPP
#define DYNAMIC_DEGRADATION_FILTER_DELTA_HPP

#include <nlopt.hpp>

#include "CollectivePerception.hpp"
#include "ExtendedKalmanFilter.hpp"

#define SENSOR_ACCURACY_INTERNAL_UNIT_FACTOR_DELTA 100.0 // the sensor accuracy is magnified internally to prevent vanishing products

class DynamicDegradationFilterDelta : public ExtendedKalmanFilter
{
public:
    enum class Variant
    {
        BinomialApproximation = 0,
        LaplaceApproximation = 1
    };

    struct BinomialApproximationParams
    {
        double FillRatioReference = -1.0;
    };

    struct LaplaceApproximationParams
    {
        double InternalUnitFactor = 1.0;

        double FillRatioReference = -1.0;

        double PredictionMean = 0.0;

        double PredictionStdDev = -1.0;

        size_t NumBlackTilesSeen = 0;

        size_t NumObservations = 0;

        void Reset();
    };

    struct TruncationParameters
    {
        DDPair StandardizedBounds = {0.0, 0.0};

        DDPair TransformedStateEstimate = {0.0, 0.0};

        double NormalizationFactor = 0.0;

        void StandardizeBounds(DDPair original_bounds, double mean, double std_dev);

        void ComputeNormalizationFactor();

        void ComputeTransformedStateEstimates();

        void Init();

        void Reset();
    };

    DynamicDegradationFilterDelta(const std::shared_ptr<CollectivePerception> &col_per_ptr,
                                  double lower_bound = 0.5 + ZERO_APPROX,
                                  double upper_bound = 1.0 - ZERO_APPROX);

    virtual void Init() override;

    virtual void Reset() override;

    virtual void Estimate();

private:
    virtual void Predict();

    virtual void Update();

    void ComputeConstrainedSensorAccuracy();

    std::shared_ptr<CollectivePerception> collective_perception_algo_ptr_;

    nlopt::opt MAP_optimizer_;

    nlopt::result MAP_optimization_status_;

    std::pair<std::vector<double>, double> MAP_outcome_ = {{-1.0}, -1.0};

    DDPair bounds_original_ = {-1.0, -1.0};

    DDPair bounds_internal_ = {-1.0, -1.0};

    TruncationParameters truncation_params_;

    BinomialApproximationParams bin_params_;

    LaplaceApproximationParams lap_params_;

    Variant variant_;

    double internal_unit_factor_ = 1.0; // internal units to prevent vanishing products

    const std::vector<double> empty_vec_double_ = {}; // dummy variable; used to pass into the predict function which doesn't require additional arguments
};

#endif