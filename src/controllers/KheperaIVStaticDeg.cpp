#include <algorithm>

#include "controllers/KheperaIVStaticDeg.hpp"
#include "algorithms/StaticDegradationFilterAlpha.hpp"
#include "algorithms/StaticDegradationFilterBravo.hpp"

KheperaIVStaticDeg::WheelTurningParams::WheelTurningParams() : TurnMech(TurningMechanism::NO_TURN),
                                                               HardTurnOnAngleThreshold(ToRadians(CDegrees(90.0))),
                                                               SoftTurnOnAngleThreshold(ToRadians(CDegrees(70.0))),
                                                               NoTurnAngleThreshold(ToRadians(CDegrees(10.0))),
                                                               MaxSpeed(10.0)
{
}

void KheperaIVStaticDeg::WheelTurningParams::Init(TConfigurationNode &xml_node)
{
    try
    {
        TurnMech = TurningMechanism::NO_TURN;
        CDegrees angle_deg;
        GetNodeAttribute(xml_node, "hard_turn_angle_threshold", angle_deg);
        HardTurnOnAngleThreshold = ToRadians(angle_deg);
        GetNodeAttribute(xml_node, "soft_turn_angle_threshold", angle_deg);
        SoftTurnOnAngleThreshold = ToRadians(angle_deg);
        GetNodeAttribute(xml_node, "no_turn_angle_threshold", angle_deg);
        NoTurnAngleThreshold = ToRadians(angle_deg);
        GetNodeAttribute(xml_node, "max_speed", MaxSpeed);
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
    }
}

/****************************************/
/****************************************/

KheperaIVStaticDeg::DiffusionParams::DiffusionParams() : GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void KheperaIVStaticDeg::DiffusionParams::Init(TConfigurationNode &xml_node)
{
    try
    {
        CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
        GetNodeAttribute(xml_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
        GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                                 ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
        GetNodeAttribute(xml_node, "delta", Delta);
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
    }
}

/****************************************/
/****************************************/

KheperaIVStaticDeg::KheperaIVStaticDeg() : ci_wheels_ptr_(NULL),
                                           ci_rab_actuator_ptr_(NULL),
                                           ci_rab_sensor_ptr_(NULL),
                                           ci_ground_ptr_(NULL),
                                           ci_proximity_ptr_(NULL),
                                           ci_leds_ptr_(NULL),
                                           sensor_degradation_filter_ptr_(NULL),
                                           collective_perception_algo_ptr_(std::make_shared<CollectivePerception>())
{
}

void KheperaIVStaticDeg::Init(TConfigurationNode &xml_node)
{
    try
    {
        /* Get pointers to devices */
        try
        {
            ci_wheels_ptr_ = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
            wheel_turning_params_.Init(GetNode(xml_node, "wheel_turning"));
            ci_rab_actuator_ptr_ = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
            ci_rab_sensor_ptr_ = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
            ci_ground_ptr_ = GetSensor<CCI_KheperaIVGroundSensor>("kheperaiv_ground");
            ci_proximity_ptr_ = GetSensor<CCI_KheperaIVProximitySensor>("kheperaiv_proximity");
            ci_leds_ptr_ = GetActuator<CCI_LEDsActuator>("leds");

            comms_params_.RABDataSize = ci_rab_actuator_ptr_->GetSize();
        }
        catch (CARGoSException &ex)
        {
            THROW_ARGOSEXCEPTION_NESTED("Error initializing the KheperaIV static degradation controller for robot \"" << GetId() << "\"", ex);
        }
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the Buzz controller for the Khepera IV", ex);
    }

    /*
     * Parse XML parameters
     */
    /* Diffusion algorithm */
    diffusion_params_.Init(GetNode(xml_node, "diffusion"));
    /* Wheel turning */
    wheel_turning_params_.Init(GetNode(xml_node, "wheel_turning"));

    // Populate robot parameters
    GetNodeAttribute(GetNode(xml_node, "ground_sensor"), "period_ticks", ground_sensor_params_.GroundMeasurementPeriodTicks);
    GetNodeAttribute(GetNode(xml_node, "ground_sensor"), "sensor_acc_b", ground_sensor_params_.ActualSensorAcc["b"]);
    GetNodeAttribute(GetNode(xml_node, "ground_sensor"), "sensor_acc_w", ground_sensor_params_.ActualSensorAcc["w"]);
    GetNodeAttribute(GetNode(xml_node, "comms"), "period_ticks", comms_params_.CommsPeriodTicks);

    // Initialize degradation filter
    TConfigurationNode &static_degradation_filter_node = GetNode(xml_node, "static_degradation_filter");

    std::string method;

    GetNodeAttribute(static_degradation_filter_node, "method", method);

    std::transform(method.begin(),
                   method.end(),
                   method.begin(),
                   ::toupper);

    if (method == "ALPHA")
    {
        sensor_degradation_filter_ptr_ =
            std::make_shared<StaticDegradationFilterAlpha>(collective_perception_algo_ptr_);
        sensor_degradation_filter_ptr_->GetParamsPtr()->Method = "ALPHA";
        sensor_degradation_filter_ptr_->GetParamsPtr()->FilterSpecificParams = {{"None", "None"}}; // no filter specific parameters for ALPHA
    }
    else if (method == "BRAVO")
    {
        sensor_degradation_filter_ptr_ =
            std::make_shared<StaticDegradationFilterBravo>(collective_perception_algo_ptr_);
        sensor_degradation_filter_ptr_->GetParamsPtr()->Method = "BRAVO";
        std::string type_2_err_prob_str;
        GetNodeAttribute(GetNode(static_degradation_filter_node, "params"), "type_2_err_prob", type_2_err_prob_str);
        sensor_degradation_filter_ptr_->GetParamsPtr()->FilterSpecificParams = {{"type_2_err_prob", type_2_err_prob_str}};
    }
    else
    {
        throw std::runtime_error("Unknown static degradation filter method, please double-check the XML file.");
    }

    GetNodeAttribute(static_degradation_filter_node, "sim", ground_sensor_params_.IsSimulated);

    SensorDegradationFilter::Params &sensor_degradation_filter_params = *sensor_degradation_filter_ptr_->GetParamsPtr();

    // Set the assumed accuracies to be the same for now; they will be updated by the loop functions if needed
    /*
        In the simulated case, the proper assumed accuracy for the robot is set by the loop functions;
        here we just initialized it to be the same as the actual accuracy but expect it to be changed before
        the experiment starts.

        In the non-simulated case, i.e., tracked ground sensor, the actual accuracy ground truth is not known and not
        required anyway, so the values obtained from the XML file for `sensor_acc_*` is used to parametrize our
        assumed accuracies. This works out because we don't simulate the tile color measurements in physical
        experiments but instead use the actual readings from the ground sensor. In other words, the values from
        `sensor_acc_*` from the XML file is our 'flawed' assumption of what the actual accuracy is.

        NOTE: in the non-simulated case, there should be no flawed robots, so the `num` attribute in the `flawed_robots`
        node (within the `static_degradation` loop functions node) should be 0. Consequently, the `acc_b` and `acc_w`
        attributes do not apply (though you should set it to be equal to the `sensor_acc_*` values in the controller
        node to facilitate data processing).
    */
    sensor_degradation_filter_params.AssumedSensorAcc["b"] = ground_sensor_params_.ActualSensorAcc["b"];
    sensor_degradation_filter_params.AssumedSensorAcc["w"] = ground_sensor_params_.ActualSensorAcc["w"];
    sensor_degradation_filter_params.RunDegradationFilter = false; // will be activated from the loop functions if required
    SetLEDs(CColor::BLUE);                                         // set to blue initially

    GetNodeAttribute(static_degradation_filter_node, "period_ticks", sensor_degradation_filter_params.FilterActivationPeriodTicks);

    /* Create a random number generator. We use the 'argos' category so
       that creation, reset, seeding and cleanup are managed by ARGoS. */
    RNG_ptr_ = CRandom::CreateRNG("argos");
}

void KheperaIVStaticDeg::Reset()
{
    tick_counter_ = 0;

    collective_perception_algo_ptr_->Reset();

    if (sensor_degradation_filter_ptr_->GetParamsPtr()->RunDegradationFilter)
    {
        sensor_degradation_filter_ptr_->Reset();

        if (sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc == ground_sensor_params_.ActualSensorAcc)
        {
            SetLEDs(CColor::GREEN); // has the correct assumed accuracy to start
        }
        else
        {
            SetLEDs(CColor::RED);
        }
    }
    else
    {
        SetLEDs(CColor::BLUE);
    }
}

std::vector<Real> KheperaIVStaticDeg::GetData() const
{
    return {static_cast<Real>(RNG_ptr_->GetSeed()),
            static_cast<Real>(collective_perception_algo_ptr_->GetParamsPtr()->NumBlackTilesSeen),
            static_cast<Real>(collective_perception_algo_ptr_->GetParamsPtr()->NumObservations),
            collective_perception_algo_ptr_->GetLocalVals().X,
            collective_perception_algo_ptr_->GetLocalVals().Confidence,
            collective_perception_algo_ptr_->GetSocialVals().X,
            collective_perception_algo_ptr_->GetSocialVals().Confidence,
            collective_perception_algo_ptr_->GetInformedVals().X,
            sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc.at("b"),
            sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc.at("w")};
}

void KheperaIVStaticDeg::SetLEDs(const CColor &color)
{
    ci_leds_ptr_->SetAllColors(color);
}

void KheperaIVStaticDeg::ControlStep()
{
    ++tick_counter_;

    // Move robot
    SetWheelSpeedsFromVector(ComputeDiffusionVector());

    // Collect ground measurement and compute local estimate
    if (tick_counter_ % ground_sensor_params_.GroundMeasurementPeriodTicks == 0)
    {
        collective_perception_algo_ptr_->GetParamsPtr()->NumBlackTilesSeen += 1 - ObserveTileColor(); // the collective perception algorithm flips the black and white tiles
        ++collective_perception_algo_ptr_->GetParamsPtr()->NumObservations;
        collective_perception_algo_ptr_->ComputeLocalEstimate(sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc.at("b"),
                                                              sensor_degradation_filter_ptr_->GetParamsPtr()->AssumedSensorAcc.at("w"));
    }

    // Communicate local estimates and compute social estimate
    if (tick_counter_ % comms_params_.CommsPeriodTicks == 0)
    {
        // Extract local estimate
        CollectivePerception::EstConfPair local_est = collective_perception_algo_ptr_->GetLocalVals();

        // Serialize data
        CByteArray data;

        data << GetId();
        data << local_est.X;
        data << local_est.Confidence;

        // Resize data
        if (comms_params_.RABDataSize > data.Size())
        {
            data.Resize(comms_params_.RABDataSize);
        }
        else
        {
            THROW_ARGOSEXCEPTION("Allocated RAB data size is too small, please increase it.");
        }

        // Broadcast data
        ci_rab_actuator_ptr_->SetData(data);

        // Listen to neighbors, if any
        CCI_RangeAndBearingSensor::TReadings packets = ci_rab_sensor_ptr_->GetReadings();

        std::vector<CollectivePerception::EstConfPair> neighbor_vals(packets.size());

        for (size_t i = 0; i < packets.size(); ++i)
        {
            packets[i].Data >> neighbor_vals[i].Id;
            packets[i].Data >> neighbor_vals[i].X;
            packets[i].Data >> neighbor_vals[i].Confidence;
        }

        // Compute social estimate only if there are neighbors (otherwise keep old social estimate so that informed estimate is still influenced)
        if (neighbor_vals.size() > 0)
        {
            collective_perception_algo_ptr_->ComputeSocialEstimate(neighbor_vals);
        }
    }

    // Compute informed estimate only if there are new local or social estimates
    if (tick_counter_ % ground_sensor_params_.GroundMeasurementPeriodTicks == 0 || tick_counter_ % comms_params_.CommsPeriodTicks == 0)
    {
        collective_perception_algo_ptr_->ComputeInformedEstimate();
    }

    // Run degradation filter
    if (sensor_degradation_filter_ptr_->GetParamsPtr()->RunDegradationFilter && tick_counter_ % sensor_degradation_filter_ptr_->GetParamsPtr()->FilterActivationPeriodTicks == 0)
    {
        sensor_degradation_filter_ptr_->Estimate();

        // update sensor accuracies
        UpdateAssumedSensorAcc(sensor_degradation_filter_ptr_->GetAccuracyEstimates());
    }
}

UInt32 KheperaIVStaticDeg::ObserveTileColor()
{
    /*
     * The ground sensors are located on the bottom of the robot, and can
     * be used to perform line following.
     *
     * The readings are in the following order (seeing the robot from TOP,
     * battery socket is the BACK):
     *
     *      front
     *
     *      0   3    r
     * l             i
     * e  1       2  g
     * f             h
     * t             t
     *
     *       back
     */

    const CCI_KheperaIVGroundSensor::TReadings &ground_readings = ci_ground_ptr_->GetReadings();

    // Use only the right sensor (index 3) to observe
    unsigned int encounter = static_cast<unsigned int>(std::round(ground_readings[3].Value));

    // Check if the ground sensor readings are actual or simulated
    if (!ground_sensor_params_.IsSimulated)
    {
        return static_cast<UInt32>(encounter);
    }
    else
    {
        // Compute simulated encounter
        float prob;

        if (encounter == 1) // white tile
        {
            prob = ground_sensor_params_.ActualSensorAcc["w"];
        }
        else if (encounter == 0) // black tile
        {
            prob = ground_sensor_params_.ActualSensorAcc["b"];
        }
        else
        {
            THROW_ARGOSEXCEPTION("Invalid tile color observed.");
        }

        // Apply noise to observation
        if (RNG_ptr_->Uniform(standard_uniform_support_) < prob) // correct observation
        {
            return static_cast<UInt32>(encounter);
        }
        else // incorrect observation
        {
            return static_cast<UInt32>(1 - encounter);
        }
    }
}

CVector2 KheperaIVStaticDeg::ComputeDiffusionVector()
{
    /* Get readings from proximity sensor */
    const CCI_KheperaIVProximitySensor::TReadings &proximity_readings = ci_proximity_ptr_->GetReadings();

    /* Sum them together */
    CVector2 diffusion_vector;

    for (size_t i = 0; i < proximity_readings.size(); ++i)
    {
        diffusion_vector += CVector2(proximity_readings[i].Value, proximity_readings[i].Angle);
    }
    /* If the angle of the vector is small enough and the closest obstacle
       is far enough, ignore the vector and go straight, otherwise return
       it */
    if (diffusion_params_.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(diffusion_vector.Angle()) &&
        diffusion_vector.Length() < diffusion_params_.Delta)
    {
        return CVector2::X * wheel_turning_params_.MaxSpeed;
    }
    else
    {
        diffusion_vector.Normalize();
        return -diffusion_vector * wheel_turning_params_.MaxSpeed;
    }
}

void KheperaIVStaticDeg::SetWheelSpeedsFromVector(const CVector2 &heading_vector)
{
    /* Get the heading angle */
    CRadians heading_angle_rad = heading_vector.Angle().SignedNormalize();

    /* Get the length of the heading vector */
    Real heading_length = heading_vector.Length();

    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real base_wheel_spd = Min<Real>(heading_length, wheel_turning_params_.MaxSpeed);

    /* State transition logic */
    if (wheel_turning_params_.TurnMech == WheelTurningParams::TurningMechanism::HARD_TURN)
    {
        if (Abs(heading_angle_rad) <= wheel_turning_params_.SoftTurnOnAngleThreshold)
        {
            wheel_turning_params_.TurnMech = WheelTurningParams::TurningMechanism::SOFT_TURN;
        }
    }

    if (wheel_turning_params_.TurnMech == WheelTurningParams::TurningMechanism::SOFT_TURN)
    {
        if (Abs(heading_angle_rad) > wheel_turning_params_.HardTurnOnAngleThreshold)
        {
            wheel_turning_params_.TurnMech = WheelTurningParams::TurningMechanism::HARD_TURN;
        }
        else if (Abs(heading_angle_rad) <= wheel_turning_params_.NoTurnAngleThreshold)
        {
            wheel_turning_params_.TurnMech = WheelTurningParams::TurningMechanism::NO_TURN;
        }
    }

    if (wheel_turning_params_.TurnMech == WheelTurningParams::TurningMechanism::NO_TURN)
    {
        if (Abs(heading_angle_rad) > wheel_turning_params_.HardTurnOnAngleThreshold)
        {
            wheel_turning_params_.TurnMech = WheelTurningParams::TurningMechanism::HARD_TURN;
        }
        else if (Abs(heading_angle_rad) > wheel_turning_params_.NoTurnAngleThreshold)
        {
            wheel_turning_params_.TurnMech = WheelTurningParams::TurningMechanism::SOFT_TURN;
        }
    }

    /* Wheel speeds based on current turning state */
    Real spd_1, spd_2;

    switch (wheel_turning_params_.TurnMech)
    {
    case WheelTurningParams::TurningMechanism::NO_TURN:
    {
        /* Just go straight */
        spd_1 = base_wheel_spd;
        spd_2 = base_wheel_spd;
        break;
    }
    case WheelTurningParams::TurningMechanism::SOFT_TURN:
    {
        /* Both wheels go straight, but one is faster than the other */
        Real fSpeedFactor = (wheel_turning_params_.HardTurnOnAngleThreshold - Abs(heading_angle_rad)) / wheel_turning_params_.HardTurnOnAngleThreshold;
        spd_1 = base_wheel_spd - base_wheel_spd * (1.0 - fSpeedFactor);
        spd_2 = base_wheel_spd + base_wheel_spd * (1.0 - fSpeedFactor);
        break;
    }
    case WheelTurningParams::TurningMechanism::HARD_TURN:
    {
        /* Opposite wheel speeds */
        spd_1 = -wheel_turning_params_.MaxSpeed;
        spd_2 = wheel_turning_params_.MaxSpeed;
        break;
    }
    }

    /* Apply the calculated speeds to the appropriate wheels */
    Real left_wheel_spd, right_wheel_spd;

    if (heading_angle_rad > CRadians::ZERO)
    {
        /* Turn Left */
        left_wheel_spd = spd_1;
        right_wheel_spd = spd_2;
    }
    else
    {
        /* Turn Right */
        left_wheel_spd = spd_2;
        right_wheel_spd = spd_1;
    }

    /* Finally, set the wheel speeds */
    left_wheel_spd = Min<Real>(left_wheel_spd, wheel_turning_params_.MaxSpeed);
    right_wheel_spd = Min<Real>(right_wheel_spd, wheel_turning_params_.MaxSpeed);

    ci_wheels_ptr_->SetLinearVelocity(left_wheel_spd, right_wheel_spd);
}

REGISTER_CONTROLLER(KheperaIVStaticDeg, "kheperaiv_static_deg_controller")