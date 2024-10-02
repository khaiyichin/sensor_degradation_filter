#ifndef KHEPERAIV_COLLECTIVE_PERCEPTION_HPP
#define KHEPERAIV_COLLECTIVE_PERCEPTION_HPP

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_ground_sensor.h>
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_proximity_sensor.h>
#include <argos3/core/utility/math/rng.h>

#include <unordered_map>

#include "algorithms/CollectivePerception.hpp"
#include "algorithms/SensorDegradationFilter.hpp"

using namespace argos;

class KheperaIVStaticDeg : public CCI_Controller
{
public:
    struct WheelTurningParams
    {
        /*
         * The turning mechanism.
         * The robot can be in three different turning states.
         */
        enum class TurningMechanism
        {
            NO_TURN = 0, // go straight
            SOFT_TURN,   // both wheels are turning forwards, but at different speeds
            HARD_TURN    // wheels are turning with opposite speeds
        };
        /*
         * Angular thresholds to change turning state.
         */
        CRadians HardTurnOnAngleThreshold;
        CRadians SoftTurnOnAngleThreshold;
        CRadians NoTurnAngleThreshold;
        /* Maximum wheel speed */
        Real MaxSpeed;
        TurningMechanism TurnMech;

        WheelTurningParams();
        void Init(TConfigurationNode &xml_node);
    };

    struct DiffusionParams
    {
        /*
         * Maximum tolerance for the proximity reading between
         * the robot and the closest obstacle.
         * The proximity reading is 0 when nothing is detected
         * and grows exponentially to 1 when the obstacle is
         * touching the robot.
         */
        Real Delta;
        /* Angle tolerance range to go straight. */
        CRange<CRadians> GoStraightAngleRange;

        /* Constructor */
        DiffusionParams();

        /* Parses the XML section for diffusion */
        void Init(TConfigurationNode &xml_node);
    };

    struct GroundSensorParams
    {
        bool IsSimulated = true;
        UInt32 GroundMeasurementPeriodTicks;
        std::unordered_map<std::string, Real> ActualSensorAcc = {{"b", -1.0}, {"w", -1.0}};
    };

    struct CommsParams
    {
        UInt32 CommsPeriodTicks;
        size_t RABDataSize;
    };

public:
    KheperaIVStaticDeg();

    virtual ~KheperaIVStaticDeg() {}

    virtual void Init(TConfigurationNode &xml_node);

    virtual void Reset();

    virtual void ControlStep();

    WheelTurningParams GetWheelTurningParams() const { return wheel_turning_params_; }

    DiffusionParams GetDiffusionParams() const { return diffusion_params_; }

    GroundSensorParams GetGroundSensorParams() const { return ground_sensor_params_; }

    CommsParams GetCommsParams() const { return comms_params_; }

    CollectivePerception::Params GetCollectivePerceptionParams() const { return *(collective_perception_algo_ptr_->GetParamsPtr()); }

    SensorDegradationFilter::Params GetSensorDegradationFilterParams() const { return *(sensor_degradation_filter_ptr_->GetParamsPtr()); }

    std::vector<Real> GetData() const;

    void UpdateAssumedSensorAcc(const std::unordered_map<std::string, Real> &updated_accuracies_map, const bool &initial = false)
    {
        sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc = updated_accuracies_map;

        if (initial)
        {
            sensor_degradation_filter_ptr_->GetParamsPtr()->InitialAssumedAcc = updated_accuracies_map;
        }
    }

    inline void SetRNGSeed(const UInt32 &seed)
    {
        RNG_ptr_->SetSeed(seed);
        RNG_ptr_->Reset();
    }

    void SetLEDs(const CColor &color);

    void ActivateDegradationFilter() { sensor_degradation_filter_ptr_->GetParamsPtr()->RunDegradationFilter = true; }

    void DeactivateDegradationFilter() { sensor_degradation_filter_ptr_->GetParamsPtr()->RunDegradationFilter = false; }

private:
    UInt32 ObserveTileColor();

    CVector2 ComputeDiffusionVector();

    void SetWheelSpeedsFromVector(const CVector2 &heading_vector);

protected:
    /* Pointer to the range and bearing actuator */
    CCI_RangeAndBearingActuator *ci_rab_actuator_ptr_;

    /* Pointer to the range and bearing sensor */
    CCI_RangeAndBearingSensor *ci_rab_sensor_ptr_;

    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator *ci_wheels_ptr_;

    /* Pointer to the ground sensor */
    CCI_KheperaIVGroundSensor *ci_ground_ptr_;

    /* Pointer to the proximity sensor */
    CCI_KheperaIVProximitySensor *ci_proximity_ptr_;

    /* Pointer to the LEDs */
    CCI_LEDsActuator *ci_leds_ptr_;

    /* Pointer to the random number generator */
    CRandom::CRNG *RNG_ptr_;

    /* Turning parameters */
    WheelTurningParams wheel_turning_params_;

    /* Diffusion parameters */
    DiffusionParams diffusion_params_;

    /* Ground sensor parameters */
    GroundSensorParams ground_sensor_params_;

    /* Communications parameters */
    CommsParams comms_params_;

    /* Collective perception algorithm */
    std::shared_ptr<CollectivePerception> collective_perception_algo_ptr_;

    /* Sensor degradation filter */
    std::shared_ptr<SensorDegradationFilter> sensor_degradation_filter_ptr_;

    UInt64 tick_counter_ = 0;

    CRange<Real> standard_uniform_support_ = CRange<Real>(0.0, 1.0);
};

#endif