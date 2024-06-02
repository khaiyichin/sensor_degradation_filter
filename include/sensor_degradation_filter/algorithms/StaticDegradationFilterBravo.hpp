#ifndef STATIC_DEGRADATION_FILTER_BRAVO_HPP
#define STATIC_DEGRADATION_FILTER_BRAVO_HPP

#include "StaticDegradationFilterAlpha.hpp"

class StaticDegradationFilterBravo : public StaticDegradationFilterAlpha
{
public:
    StaticDegradationFilterBravo(const std::shared_ptr<CollectivePerception> &col_per_ptr);

    virtual void Init() override;

    virtual void Estimate() override;

protected:

    /**
     * @brief Compare the difference in local estimates using a confidence interval
     * 
     * This is used to self-identify whether the filter should be activated
     * 
     * @return true if the neighbors' estimates are close enough to the self-estimate
     * @return false otherwise
     */
    bool CompareWithNeighborEstimates();

    double type_2_err_prob_; ///< Parameter to adjust the width of the confidence interval
};

#endif