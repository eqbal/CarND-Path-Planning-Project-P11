#ifndef HIGHWAY_MAP_H
#define HIGHWAY_MAP_H

#include <iostream>
#include <vector>

struct HighwayMap {
    std::vector<Lane> lanes;

    /**
     * @brief Default constructor.
     */
    HighwayMap();

    /**
     * @brief Return the indexes to the lane(s) adjacent to the given one.
     */
    std::vector<size_t> adjacentLanes(size_t lane) const;

    /**
     * @brief Return the index of the lane closest to the given point.
     */
    size_t closestIndex(double x, double y) const;

    /**
     * @brief Return the index of the lane closest to the given state.
     */
    size_t closestIndex(const State &state) const;

    /**
     * @brief Return the index of the lane closest to the given point.
     */
    size_t closestIndex(double d) const;

    /**
     * @brief Return a reference to the lane closest to the given point.
     */
    const Lane &closestLane(double x, double y) const;

    /**
     * @brief Return the number of lanes in the highway.
     */
    size_t size() const;
};

std::istream &operator >> (std::istream &in, HighwayMap &highway);

#endif
