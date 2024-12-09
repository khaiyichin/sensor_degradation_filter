// Test integration functionality of the ELBO class
#include "algorithms/ELBO.hpp"

#include <nlopt.hpp>
#include <iomanip>
#include <iostream> // debugging purpose

#define INTEGRATION_EPSILON 1e-9
#define OPTIMIZATION_EPSILON 1e-3

int main()
{
    // Initialize with integration limits of 0.5 and 1.0
    SensorAccuracyELBO elbo(0.5, 1.0);

    // Populate parameters of distributions in the ELBO
    auto params_ptr = elbo.GetDistributionParametersPtr();

    params_ptr->PredictionMean = 0.95;
    params_ptr->PredictionStdDev = std::sqrt(0.005);
    params_ptr->FillRatio = 0.9;
    params_ptr->PredictionLowerBound = 0.5;
    params_ptr->PredictionUpperBound = 1.0;
    params_ptr->SurrogateLoc = 0.998001;
    params_ptr->SurrogateScale = std::sqrt(0.001998);
    params_ptr->SurrogateLowerBound = 0.5;
    params_ptr->SurrogateUpperBound = 1.0;
    params_ptr->NumBlackTilesSeen = 0;
    params_ptr->NumObservations = 1;

    elbo.ComputeELBO();

    double expected_result_1 = -2.51325848473;

    if (std::abs(elbo.GetELBOResult() - expected_result_1) > INTEGRATION_EPSILON)
    {
        std::cout << "Integration 1 Error!" << std::endl;
    }
    else
    {
        std::cout << "Integration 1 Correct!" << std::endl;
    }

    std::cout << "Computed = " << elbo.GetELBOResult() << std::endl;
    std::cout << "Expected = " << expected_result_1 << std::endl;
    std::cout << std::endl;

    // Modify and compute new integration result
    params_ptr->PredictionMean = 0.768;
    params_ptr->PredictionStdDev = std::sqrt(0.025);
    params_ptr->FillRatio = 0.99;
    params_ptr->PredictionLowerBound = 0.5;
    params_ptr->PredictionUpperBound = 1.0;
    params_ptr->SurrogateLoc = 0.88;
    params_ptr->SurrogateScale = std::sqrt(0.099);
    params_ptr->SurrogateLowerBound = 0.5;
    params_ptr->SurrogateUpperBound = 1.0;
    params_ptr->NumBlackTilesSeen = 25;
    params_ptr->NumObservations = 47;

    elbo.ComputeELBO();

    double expected_result_2 = -14.5533344518;

    if (std::abs(elbo.GetELBOResult() - expected_result_2) > INTEGRATION_EPSILON)
    {
        std::cout << "Integration 2 Error!" << std::endl;
    }
    else
    {
        std::cout << "Integration 2 Correct!" << std::endl;
    }

    std::cout << "Computed = " << elbo.GetELBOResult() << std::endl;
    std::cout << "Expected = " << expected_result_2 << std::endl;
    std::cout << std::endl;

    // Modify and compute MAP optimization result
    params_ptr->PredictionMean = 0.9940172181925864;
    params_ptr->PredictionStdDev = std::sqrt(0.10000493386913718);
    params_ptr->FillRatio = 0.7481573110960751;
    params_ptr->PredictionLowerBound = 0.5;
    params_ptr->PredictionUpperBound = 1.0;
    params_ptr->NumBlackTilesSeen = 36;
    params_ptr->NumObservations = 50;

    // Initialize NLopt objects
    nlopt::opt opt(nlopt::LN_NELDERMEAD, 1);

    auto nlopt_wrapped_MAP_fcn = [](const std::vector<double> &x,
                                    std::vector<double> &grad,
                                    void *my_func_data) -> double
    {
        // Cast into SensorAccuracyELBO type object
        SensorAccuracyELBO *elbo_obj_ptr = static_cast<SensorAccuracyELBO *>(my_func_data);

        elbo_obj_ptr->ComputePredictionNormalizationConstant();

        return joint_density_fcn(x[0], elbo_obj_ptr->GetDistributionParametersPtr().get());
    };

    opt.set_max_objective(nlopt_wrapped_MAP_fcn, &elbo);
    opt.set_lower_bounds(0.5);
    opt.set_upper_bounds(1.0);
    opt.set_xtol_abs(1e-9);

    std::vector<double> x_MAP{params_ptr->PredictionMean};
    double obj_val, expected_mean = 0.95044891;

    /* Optimization flags (from nlopt.hpp):

        enum result {
            FAILURE = -1,         // generic failure code
            INVALID_ARGS = -2,
            OUT_OF_MEMORY = -3,
            ROUNDOFF_LIMITED = -4,
            FORCED_STOP = -5,
            SUCCESS = 1,          // generic success code
            STOPVAL_REACHED = 2,
            FTOL_REACHED = 3,
            XTOL_REACHED = 4,
            MAXEVAL_REACHED = 5,
            MAXTIME_REACHED = 6,
            NUM_RESULTS           // not a result, just the number of them
        };

    */

    try
    {
        nlopt::result result = opt.optimize(x_MAP, obj_val);
        if (std::abs(x_MAP[0] - expected_mean) > OPTIMIZATION_EPSILON)
        {
            std::cout << "Optimization (mean) Error!" << std::endl;
        }
        else
        {
            std::cout << "Optimization (mean) Correct!" << std::endl;
        }
        std::cout << "Computed = " << x_MAP[0] << std::endl;
        std::cout << "Expected = " << expected_mean << std::endl;
        std::cout << "found maximum at f(" << x_MAP[0] << ") = "
                  << std::setprecision(10) << obj_val << std::endl;
        std::cout << std::endl;
    }
    catch (std::exception &e)
    {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    /*
     */

    auto nlopt_wrapped_ELBO_fcn = [](const std::vector<double> &x,
                                     std::vector<double> &grad,
                                     void *my_func_data) -> double
    {
        // Cast into SensorAccuracyELBO type object
        SensorAccuracyELBO *elbo_obj_ptr = static_cast<SensorAccuracyELBO *>(my_func_data);

        auto params_ptr = elbo_obj_ptr->GetDistributionParametersPtr();

        params_ptr->SurrogateScale = x[0];

        elbo_obj_ptr->ComputeELBO();

        return elbo_obj_ptr->GetELBOResult();
    };

    // Modify and compute ELBO optimization result to estimate the surrogate standard deviation
    params_ptr->PredictionMean = 0.9940172181925864;
    params_ptr->PredictionStdDev = std::sqrt(0.10000493386913718);
    params_ptr->FillRatio = 0.794046296702686;
    params_ptr->PredictionLowerBound = 0.5;
    params_ptr->PredictionUpperBound = 1.0;
    params_ptr->NumBlackTilesSeen = 39;
    params_ptr->NumObservations = 50;
    params_ptr->SurrogateLoc = 0.97763716;
    params_ptr->SurrogateScale = -1.0;
    params_ptr->SurrogateLowerBound = 0.5;
    params_ptr->SurrogateUpperBound = 1.0;

    opt.set_max_objective(nlopt_wrapped_ELBO_fcn, &elbo);
    opt.set_lower_bounds(std::sqrt(1e-3));
    opt.set_upper_bounds(HUGE_VAL);

    std::vector<double> x_ELBO{params_ptr->PredictionStdDev};
    double expected_std_dev = std::sqrt(0.01093531);

    try
    {
        nlopt::result result = opt.optimize(x_ELBO, obj_val);
        if (std::abs(x_ELBO[0] - expected_std_dev) > OPTIMIZATION_EPSILON)
        {
            std::cout << "Optimization (standard deviation) Error!" << std::endl;
        }
        else
        {
            std::cout << "Optimization (standard deviation) Correct!" << std::endl;
        }
        std::cout << "Computed = " << x_ELBO[0] << std::endl;
        std::cout << "Expected = " << expected_std_dev << std::endl;
        std::cout << "found maximum at f(" << x_ELBO[0] << ") = "
                  << std::setprecision(10) << obj_val << std::endl;
        std::cout << std::endl;
    }
    catch (std::exception &e)
    {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    return 0;
}