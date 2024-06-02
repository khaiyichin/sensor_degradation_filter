#include "algorithms/StaticDegradationFilterBravo.hpp"
#include <gsl/gsl_statistics_double.h>
#include <gsl/gsl_cdf.h>

StaticDegradationFilterBravo::StaticDegradationFilterBravo(const std::shared_ptr<CollectivePerception> &col_per_ptr)
    : StaticDegradationFilterAlpha(col_per_ptr) {}

void StaticDegradationFilterBravo::Init()
{
    type_2_err_prob_ = std::stod(params_ptr_->FilterSpecificParams["type_2_err_prob"].c_str());
}

#include <iostream>
bool StaticDegradationFilterBravo::CompareWithNeighborEstimates()
{
    std::vector<CollectivePerception::EstConfPair> neighbor_estconf_vec = collective_perception_algo_ptr_->GetParamsPtr()->MostRecentNeighborEstimates;

    // Get the number of neighbors
    size_t num_neighbors = neighbor_estconf_vec.size();

    if (num_neighbors <= 1)
    {
        return false; // not enough neighbor data to decide if filter needs to be run, so maintaining status quo
    }

    // Extract the neighbors' local estimates (don't need the confidence values)
    std::vector<double> neighbors_x_hat_vec(num_neighbors);

    for (size_t i = 0; i < num_neighbors; ++i)
    {
        neighbors_x_hat_vec[i] = neighbor_estconf_vec[i].X;
    }

    // Compute sample mean
    double sample_mean = gsl_stats_mean(neighbors_x_hat_vec.data(), 1, num_neighbors);

    // Compute sample standard deviation
    double sample_std_dev = gsl_stats_sd(neighbors_x_hat_vec.data(), 1, num_neighbors);

    // Compute T-score
    double t_score = gsl_cdf_tdist_Qinv(
        (1.0 - type_2_err_prob_) / 2.0,
        num_neighbors);

    // Compute one-sided interval
    double one_sided_interval = sample_std_dev * (t_score / std::sqrt(num_neighbors) + 1.0);

    // Check to see if the absolute difference is within bounds
    if (std::abs(sample_mean - collective_perception_algo_ptr_->GetLocalVals().X) < one_sided_interval)
    {
        return false; // neighbors estimate are similar enough with self-estimate
    }
    else
    {
        return true;
    }
}

void StaticDegradationFilterBravo::Estimate()
{
    // Self-identify whether it's different enough
    if (CompareWithNeighborEstimates())
    {
        StaticDegradationFilterAlpha::Estimate();
    }
}