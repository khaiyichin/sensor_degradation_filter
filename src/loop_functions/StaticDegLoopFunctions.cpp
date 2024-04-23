#include "loop_functions/StaticDegLoopFunctions.hpp"

void StaticDegLoopFunctions::Init(TConfigurationNode &t_tree)
{
    // Extract XML information
    try
    {
        // Call parent's Init
        CLoopFunctions::Init(t_tree);

        // Grab the reference to the XML node with the tag "static_degradation"
        TConfigurationNode &static_deg_node = GetNode(t_tree, "static_degradation");

        // Grab arena information
        TConfigurationNode &arena_tiles_node = GetNode(static_deg_node, "arena_tiles");

        // Grab verbosity level
        GetNodeAttribute(GetNode(static_deg_node, "verbosity"), "level", verbose_level_);

        // Get a pointer to the ARGoS floor entity (method provided by superclass)
        space_ptr_ = &GetSpace();
        floor_entity_ptr_ = &space_ptr_->GetFloorEntity();

        // Get the size of the arena (in units of tiles)
        UInt64 arena_x, arena_y;

        GetNodeAttribute(arena_tiles_node, "tile_count_x", arena_x);
        GetNodeAttribute(arena_tiles_node, "tile_count_y", arena_y);

        arena_tile_count_ = std::make_pair(arena_x, arena_y);

        // Get the limits of arena
        CRange<CVector3> lims = space_ptr_->GetArenaLimits();
        arena_lower_lim_ = std::make_pair(static_cast<Real>(lims.GetMin().GetX()),
                                          static_cast<Real>(lims.GetMin().GetY()));

        // Compute tile size
        CVector3 arena_size = space_ptr_->GetArenaSize();

        Real length_x = arena_size.GetX() / arena_x; // tile size in the x-direction
        Real length_y = arena_size.GetY() / arena_y; // tile size in the y-direction

        assert(length_x == length_y); // only square tiles allowed
        arena_tile_size_ = length_x;

        // Grab the constrained area to compute the true swarm density
        auto &box_map = space_ptr_->GetEntitiesByType("box");

        // Get constrained x length
        CBoxEntity &wall_west = *any_cast<CBoxEntity *>(box_map["wall_west"]);
        CBoxEntity &wall_east = *any_cast<CBoxEntity *>(box_map["wall_east"]);

        Real wall_west_thickness = wall_west.GetSize().GetX();
        Real wall_east_thickness = wall_east.GetSize().GetX();

        Real wall_west_pos_x = wall_west.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
        Real wall_east_pos_x = wall_east.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();

        assert(abs(wall_west_pos_x) == abs(wall_east_pos_x));         // ensure that walls are evenly separated
        assert(abs(wall_west_thickness) == abs(wall_east_thickness)); // ensure that walls are equally thick

        // Get constrained y length
        CBoxEntity &wall_north = *any_cast<CBoxEntity *>(box_map["wall_north"]);
        CBoxEntity &wall_south = *any_cast<CBoxEntity *>(box_map["wall_south"]);

        Real wall_north_thickness = wall_north.GetSize().GetY();
        Real wall_south_thickness = wall_south.GetSize().GetY();

        Real wall_north_pos_y = wall_north.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
        Real wall_south_pos_y = wall_south.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();

        assert(abs(wall_north_pos_y) == abs(wall_south_pos_y));         // ensure that walls are evenly separated
        assert(abs(wall_north_thickness) == abs(wall_south_thickness)); // ensure that walls are equally thick

        // Compute constrained arena area
        Real constrained_x_distance = (wall_east_pos_x - wall_west_pos_x) - wall_west_thickness;
        Real constrained_y_distance = (wall_north_pos_y - wall_south_pos_y) - wall_north_thickness;

        Real constrained_area = constrained_x_distance * constrained_y_distance;

        // Grab number of trials
        GetNodeAttribute(GetNode(static_deg_node, "num_trials"), "value", exp_params_.NumTrials);

        // Grab target fill ratio
        GetNodeAttribute(GetNode(static_deg_node, "target_fill_ratio"), "value", exp_params_.TargetFillRatio);

        // Grab random KheperaIV static degradation controller
        auto &kheperaiv_entities_map = space_ptr_->GetEntitiesByType("kheperaiv");
        CKheperaIVEntity &random_kheperaiv_entity = *any_cast<CKheperaIVEntity *>(kheperaiv_entities_map.begin()->second);
        KheperaIVStaticDeg &static_deg_controller = dynamic_cast<KheperaIVStaticDeg &>(random_kheperaiv_entity.GetControllableEntity().GetController());

        // Grab sensor accuracies
        exp_params_.ActualSensorAcc = static_deg_controller.GetGroundSensorParams().ActualSensorAcc;

        TConfigurationNode &flawed_robots_node = GetNode(static_deg_node, "flawed_robots");
        GetNodeAttribute(flawed_robots_node, "num", exp_params_.NumFlawedRobots);
        GetNodeAttribute(flawed_robots_node, "acc_b", exp_params_.AssumedSensorAcc.at("b"));
        GetNodeAttribute(flawed_robots_node, "acc_w", exp_params_.AssumedSensorAcc.at("w"));

        // Grab robot speeds
        exp_params_.RobotSpeed = static_deg_controller.GetWheelTurningParams().MaxSpeed;

        // Grab sensing and communications period
        exp_params_.MeasurementPeriod = static_deg_controller.GetGroundSensorParams().GroundMeasurementPeriodTicks;
        exp_params_.CommsPeriod = static_deg_controller.GetCommsParams().CommsPeriodTicks;

        // Grab number of agents and communications range
        UInt64 size;
        Real range;

        CRABEquippedEntity &random_rab = random_kheperaiv_entity.GetRABEquippedEntity();
        size = kheperaiv_entities_map.size();
        range = random_rab.GetRange();

        exp_params_.NumRobots = size;   // the number of range and bearing sensors is the same as the number of robots
        exp_params_.CommsRange = range; // all the range and bearing sensors have the same range
        exp_params_.Density = static_cast<Real>(exp_params_.NumRobots) * M_PI * std::pow(exp_params_.CommsRange, 2) /
                              constrained_area; // the density is the ratio of swarm communication area to total walkable area

        // Grab number of steps
        exp_params_.NumSteps = GetSimulator().GetMaxSimulationClock();

        // Grab number of ticks in a second
        TConfigurationNode &framework_experiment_node = GetNode(GetNode(GetSimulator().GetConfigurationRoot(), "framework"), "experiment");
        GetNodeAttribute(framework_experiment_node, "ticks_per_second", ticks_per_sec_);

        // Grab save path
        GetNodeAttribute(GetNode(static_deg_node, "path"), "folder", exp_params_.SaveFolder);

        // Check if folder exists
        if (!std::filesystem::exists(exp_params_.SaveFolder))
        {
            THROW_ARGOSEXCEPTION("Save folder doesn't exist.");
        }

        if (verbose_level_ == "full" || verbose_level_ == "reduced")
        {
            LOG << "[INFO] Verbose level = \"" << verbose_level_ << "\"" << std::endl;
            LOG << "[INFO] Specifying number of arena tiles = " << arena_x << "*" << arena_y << std::endl;
            LOG << "[INFO] Specifying number of robots = " << exp_params_.NumRobots << std::endl;
            LOG << "[INFO] Specifying number of flawed robots = " << exp_params_.NumFlawedRobots << std::endl;
            LOG << "[INFO] Specifying number of trials = " << exp_params_.NumTrials << std::endl;
            LOG << "[INFO] Specifying number of simulation steps = " << exp_params_.NumSteps << std::endl;
            LOG << "[INFO] Specifying target fill ratio = " << exp_params_.TargetFillRatio << std::endl;
            LOG << "[INFO] Specifying flawed assumed accuracies (b,w) = " << exp_params_.AssumedSensorAcc["b"] << "," << exp_params_.AssumedSensorAcc["w"] << std::endl;
            LOG << "[INFO] Specifying actual accuracies (b,w) = " << exp_params_.ActualSensorAcc["b"] << "," << exp_params_.ActualSensorAcc["w"] << std::endl;
            LOG << "[INFO] Specifying communication range = " << exp_params_.CommsRange << " m" << std::endl;
            LOG << "[INFO] Specifying communication period = " << exp_params_.CommsPeriod << " ticks" << std::endl;
            LOG << "[INFO] Specifying measurement period = " << exp_params_.MeasurementPeriod << " ticks" << std::endl;
            LOG << "[INFO] Specifying robot speed = " << exp_params_.RobotSpeed << " cm/s" << std::endl;
            LOG << "[INFO] Computed swarm density = " << exp_params_.Density << std::endl;
            LOG << "[INFO] Specifying output folder = \"" << exp_params_.SaveFolder << "\"" << std::endl;
            LOG << "[INFO] Generated tile size = " << arena_tile_size_ << " m" << std::endl;
            LOG << "[INFO] Running trial 1" << std::endl;
        }
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }

    // Iterate through all the robots and modify the flawed robots assumed accuracies
    auto &kheperaiv_entities_map = space_ptr_->GetEntitiesByType("kheperaiv");
    for (auto itr = kheperaiv_entities_map.begin(); itr != kheperaiv_entities_map.end(); ++itr)
    {
        sorted_robot_ids_.push_back(itr->first);
    }

    std::cout << "debug: IDs before =";

    for (auto itr = sorted_robot_ids_.begin(); itr != sorted_robot_ids_.end(); ++itr)
    {
        std::cout << *itr;
    }
    std::cout << std::endl;

    // Sort the robots based on their IDs so that the last n number of robots are flawed
    std::sort(sorted_robot_ids_.begin(), sorted_robot_ids_.end());
    std::cout << "debug: IDs after =";

    for (auto itr = sorted_robot_ids_.begin(); itr != sorted_robot_ids_.end(); ++itr)
    {
        std::cout << *itr;
    }
    std::cout << std::endl;

    // Go through the last n number from the back to adjust their assumed sensor accuracies
    for (size_t i = exp_params_.NumRobots - 1; i >= (exp_params_.NumRobots - exp_params_.NumFlawedRobots); --i)
    {
        CKheperaIVEntity &kheperaiv_entity = *any_cast<CKheperaIVEntity *>(kheperaiv_entities_map[sorted_robot_ids_[i]]);
        KheperaIVStaticDeg &controller = dynamic_cast<KheperaIVStaticDeg &>(kheperaiv_entity.GetControllableEntity().GetController());

        std::cout << "debug setting flawed sensor i=" << i << std::endl;

        controller.UpdateAssumedSensorAcc(exp_params_.AssumedSensorAcc, true);
        controller.ActivateDegradationFilter();
        controller.SetLEDs(CColor::RED);
    }

    // Setup experiment
    SetupExperiment();
}

void StaticDegLoopFunctions::SetupExperiment()
{
    // Create new Arena object
    arena_ = Arena(arena_tile_count_, arena_lower_lim_, arena_tile_size_, exp_params_.TargetFillRatio, GetSimulator().GetRandomSeed());

    id_data_str_map_ = RobotIdDataStrMap();

    InitializeJSON();
}

void StaticDegLoopFunctions::PostStep()
{
    // Grab data from each robot
    CSpace::TMapPerType &kheperaiv_entities_map = GetSpace().GetEntitiesByType("kheperaiv");

    for (CSpace::TMapPerType::iterator itr = kheperaiv_entities_map.begin(); itr != kheperaiv_entities_map.end(); ++itr)
    {
        // Get reference to the robot's controller
        CKheperaIVEntity &kheperaiv_entity = *any_cast<CKheperaIVEntity *>(itr->second);
        KheperaIVStaticDeg &controller = dynamic_cast<KheperaIVStaticDeg &>(kheperaiv_entity.GetControllableEntity().GetController());

        // Convert and store data string
        id_data_str_map_[kheperaiv_entity.GetId()].push_back(ConvertDataToString(controller.GetData()));
    }
}

void StaticDegLoopFunctions::PostExperiment()
{
    // Store new trial result
    std::vector<std::vector<std::string>> vec;
    vec.reserve(exp_params_.NumRobots);

    for (auto itr = sorted_robot_ids_.begin(); itr != sorted_robot_ids_.end(); ++itr)
    {
        vec.push_back(id_data_str_map_.at(*itr)); // store the values from the map, which are vectors of strings
    }

    curr_json_["data_str"] = vec;

    json_data_vec_.push_back(curr_json_);

    // Determine whether to terminate simulation
    if (++curr_trial_ind_ % exp_params_.NumTrials == 0) // all trials have completed
    {
        if (verbose_level_ == "full" || verbose_level_ == "reduced")
        {
            LOG << "[INFO] All trials completed." << std::endl;
        }

        SaveData();

        finished_ = true;
    }
    else // more trials required
    {
        // Repeat trial
        if (verbose_level_ == "full")
        {
            LOG << "[INFO] Running trial " << curr_trial_ind_ + 1 << std::endl;
        }
    }
}

void StaticDegLoopFunctions::SaveData()
{
    // Write JSON files into folder
    std::string filepath_prefix = exp_params_.SaveFolder + "/";

    // Create individual JSON files
    for (size_t i = 0; i < json_data_vec_.size(); ++i)
    // for (auto itr = json_data_vec_.begin(); itr != json_data_vec_.end(); ++itr)
    {
        std::string filename = "flw" + std::to_string(exp_params_.NumFlawedRobots) + "_" +
                               "t" + std::to_string(i);

        filename = filepath_prefix + filename + ".json";

        // Export to single JSON file
        std::ofstream outfile(filename);

        outfile << std::setw(4) << (json_data_vec_[i]) << std::endl; // write pretty JSON

        outfile.close();
    }

    // benchmark_algo_ptr_->SaveData(foldername_prefix);
}

void StaticDegLoopFunctions::InitializeJSON()
{
    curr_json_ = ordered_json{};
    curr_json_["sim_type"] = "dynamic_topo_static_deg";
    curr_json_["num_robots"] = exp_params_.NumRobots;
    curr_json_["num_flawed_robots_"] = exp_params_.NumFlawedRobots;
    curr_json_["num_trials"] = exp_params_.NumTrials;
    curr_json_["num_steps"] = exp_params_.NumSteps;
    curr_json_["tfr"] = exp_params_.TargetFillRatio;
    curr_json_["flawed_sensor_acc_b"] = exp_params_.AssumedSensorAcc["b"];
    curr_json_["flawed_sensor_acc_w"] = exp_params_.AssumedSensorAcc["w"];
    curr_json_["correct_sensor_acc_b"] = exp_params_.ActualSensorAcc["b"];
    curr_json_["correct_sensor_acc_w"] = exp_params_.ActualSensorAcc["w"];
    curr_json_["comms_range"] = exp_params_.CommsRange;
    curr_json_["comms_period"] = exp_params_.CommsPeriod;
    curr_json_["meas_period"] = exp_params_.MeasurementPeriod;
    curr_json_["speed"] = exp_params_.RobotSpeed;
    curr_json_["density"] = exp_params_.Density;
    curr_json_["trial_ind"] = curr_trial_ind_;
}

CColor StaticDegLoopFunctions::GetFloorColor(const CVector2 &c_position_on_plane)
{
    UInt32 color_int = arena_.GetColor(c_position_on_plane.GetX(), c_position_on_plane.GetY());

    return color_int == 1 ? CColor::BLACK : CColor::WHITE;
}

std::string StaticDegLoopFunctions::ConvertDataToString(const std::vector<Real> &data)
{
    std::stringstream ss;
    ss.precision(3);
    ss << std::fixed;

    for (auto itr = data.begin(); itr != data.end(); ++itr)
    {
        ss << *itr << ",";
    }

    ss.str().pop_back(); // remove the last comma

    return ss.str();
}

REGISTER_LOOP_FUNCTIONS(StaticDegLoopFunctions, "static_deg_loop_functions")