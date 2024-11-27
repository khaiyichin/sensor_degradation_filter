#ifndef ELBO_HPP
#define ELBO_HPP

#include <utility>
#include <memory>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_integration.h>

struct DistributionParameters
{
    void Reset()
    {
        PredictionMean = -1.0;
        PredictionVariance = -1.0;
        PredictionNormConst = -1.0;
        FillRatio = -1.0;
        SurrogateMean = -1.0;
        SurrogateVariance = -1.0;
        NumBlackTilesSeen = -1;
        NumObservations = -1;
    }

    double PredictionMean;

    double PredictionVariance;

    double PredictionNormConst;

    double FillRatio;

    double PredictionLowerBound;

    double PredictionUpperBound;

    double SurrogateMean;

    double SurrogateVariance;

    double SurrogateLowerBound;

    double SurrogateUpperBound;

    size_t NumBlackTilesSeen;

    size_t NumObservations;
};

// Declare generic PDF functions for global usage (NLopt wants a C-style function pointer, which makes member functions more complicated to use)
double surrogate_trunc_normal_pdf(double x, void *params);
double prediction_trunc_normal_pdf(double x, void *params);
double observation_likelihood_binomial_pdf(double x, void *params);
double joint_density_fcn(double x, void *params);
double ELBO_integrand(double x, void *params);

class ELBO
{
public:
    ELBO(double a, double b, size_t n = 1000);

    ~ELBO();

    void Reset();

    /**
     * @brief Compute the normalization constant for the prediction model.
     * This is provided for the prediction model (but not for the surrogate model)
     * because it stays constant every step (unlike when we optimize the ELBO).
     *
     */
    void ComputePredictionNormalizationConstant();

    void ComputeELBO();

    void SetIntegrationLimits(double a, double b) { integration_limits_ = {a, b}; }

    std::pair<double, double> GetIntegrationLimits() const { return integration_limits_; }

    double GetELBOResult() const { return integration_outputs_.first; }

    double GetELBOError() const { return integration_outputs_.second; }

    std::shared_ptr<DistributionParameters> GetDistributionParametersPtr() { return integration_parameters_ptr_; }

private:
    size_t workspace_size_;

    gsl_integration_workspace *workspace_;

    gsl_function integrand_;

    std::shared_ptr<DistributionParameters> integration_parameters_ptr_;

    std::pair<double, double> integration_limits_; // lower and upper limit

    std::pair<double, double> integration_outputs_; // result and error
};

#endif