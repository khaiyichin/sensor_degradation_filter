#ifndef STATIC_DEG_LOOP_FUNCTIONS_HPP
#define STATIC_DEG_LOOP_FUNCTIONS_HPP

#include <filesystem>
#include <algorithm>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>

#include "json.hpp"
#include "Arena.hpp"
#include "controllers/KheperaIVStaticDeg.hpp"

using namespace argos;
using namespace nlohmann;

using RobotIdDataStrMap = std::unordered_map<std::string, std::vector<std::string>>;

struct ExperimentParams
{
    UInt32 NumRobots;

    UInt32 NumFlawedRobots;

    UInt32 NumTrials;

    UInt32 CommsPeriod;

    UInt32 MeasurementPeriod;

    UInt64 NumSteps;

    std::unordered_map<std::string, Real> ActualSensorAcc = {{"b", -1.0}, {"w", -1.0}};

    std::unordered_map<std::string, Real> AssumedSensorAcc = {{"b", -1.0}, {"w", -1.0}};

    Real CommsRange;

    Real RobotSpeed;

    Real Density;

    Real TargetFillRatio;

    std::string SaveFolder;
};

class StaticDegLoopFunctions : public CLoopFunctions
{
public:
    /**
     * @brief Initialize loop functions
     *
     * @param t_tree Pointer to the XML config node
     */
    virtual void Init(TConfigurationNode &t_tree);

    /**
     * @brief Reset loop functions (triggered by simulation reset)
     *
     */
    inline void Reset() { SetupExperiment(); }

    /**
     * @brief Setup experiment
     *
     */
    void SetupExperiment();

    /**
     * @brief Execute post step activities
     *
     */
    virtual void PostStep();

    /**
     * @brief Execute post experiment activities
     *
     */
    virtual void PostExperiment();

    /**
     * @brief Write data to disk
     *
     */
    void SaveData();

    /**
     * @brief Check if experiment is over
     *
     * @return true
     * @return false
     */
    inline bool IsExperimentFinished() { return finished_; }

    /**
     * @brief Get the floor color
     *
     * @param c_position_on_plane Coordinates of the floor
     * @return CColor Color at the specified coordinates
     */
    virtual CColor GetFloorColor(const CVector2 &c_position_on_plane);

    std::string ConvertDataToString(const std::vector<Real> &data);

private:
    void InitializeJSON();

    UInt32 curr_trial_ind_ = 0; ///< Counter to keep track of trials

    ExperimentParams exp_params_;

    Real arena_tile_size_;

    Real ticks_per_sec_; ///< Number of ticks in one second

    std::pair<UInt64, UInt64> arena_tile_count_;

    std::pair<Real, Real> arena_lower_lim_;

    Arena arena_; ///< Arena object

    CFloorEntity *floor_entity_ptr_ = NULL; ///< Pointer to the floor entity class

    CSpace *space_ptr_ = NULL; ///< Pointer to the space class

    std::string verbose_level_; ///< Output verbosity level

    std::string output_filename_; ///< Filename of output data

    ordered_json curr_json_; ///< Current JSON data object

    std::vector<ordered_json> json_data_vec_; ///< Vector of JSON data objects

    RobotIdDataStrMap id_data_str_map_;

    std::vector<std::string> sorted_robot_ids_;

    bool finished_ = false; ///< Flag to indicate whether all simulation parameters have been executed
};

#endif