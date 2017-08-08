#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include "json.hpp"

#include <Eigen/Dense>

#include <cstddef>
#include <vector>

struct Waypoints {
    /** @brief Waypoint horizontal coordinates. */
    std::vector<double> x;

    /** @brief Waypoint vertical coordinates. */
    std::vector<double> y;

    /**
     * @brief Default constructor.
     */
    Waypoints();

    /**
     * @brief Create a new waypoint list with given coordinate vectors.
     */
    Waypoints(const std::vector<double> &x, const std::vector<double> &y);

    /**
     * @brief Computes polynomial coefficients for these waypoints.
     */
    Eigen::VectorXd fit() const;

    /**
     * @brief Update this waypoint list with data from the given JSON object.
     */
    void update(const nlohmann::json &json);

    /**
     * @brief Return the number of waypoints.
     */
    size_t size() const;
};

/**
 * @brief Evaluate the given polynomial at the given position.
 */
double eval(const Eigen::VectorXd &a, double x);

#endif
