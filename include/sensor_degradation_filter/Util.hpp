#ifndef UTIL_HPP
#define UTIL_HPP

#include <vector>
#include <string>
#include <ctime>
#include <random>    // mt19937, uniform_real_distribution, normal_distribution
#include <algorithm> // sample
#include <stdexcept>

template <typename T>
std::vector<T> GenerateLinspace(const T &min, const T &max, const size_t &steps)
{
    // Compute increment
    T inc = (max - min) / static_cast<T>(steps - 1);

    // Populate vector
    std::vector<T> output(steps);
    T val;

    for (auto itr = output.begin(), val = min; itr != output.end(); ++itr, val += inc)
    {
        *itr = val;
    }

    return output;
}

inline std::string GetCurrentTimeStr()
{
    // Grab current local time
    time_t curr_time;
    time(&curr_time);
    tm *curr_tm = localtime(&curr_time);

    std::string datetime;
    datetime.resize(100);

    // Convert to string
    strftime(&(datetime[0]), datetime.size(), "%m%d%y_%H%M%S", curr_tm);
    return std::string(datetime.c_str());
}

inline std::string Round1000DoubleToStr(const double &val)
{
    return std::to_string(static_cast<int>(std::round(val * 1e3)));
}

/**
 * @brief Draw a sample of robot IDs without replacement
 *
 * @param num_robots_to_sample Number of robots to sample
 * @param robot_id_vec Vector of robot IDs to sample from
 * @return std::vector<std::string> Drawn robot IDs
 */
inline std::vector<std::string> SampleRobotIdsWithoutReplacement(const unsigned int &num_robots_to_sample,
                                                                 const std::vector<std::string> &robot_id_vec)
{
    // Sample random robot IDs (without replacement)
    std::vector<std::string> sampled_robot_ids;

    std::sample(robot_id_vec.begin(),
                robot_id_vec.end(),
                std::back_inserter(sampled_robot_ids),
                num_robots_to_sample,
                std::mt19937{std::random_device{}()});

    return sampled_robot_ids;
}

#endif