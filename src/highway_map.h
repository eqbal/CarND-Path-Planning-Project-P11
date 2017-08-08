#ifndef HIGHWAY_MAP_H
#define HIGHWAY_MAP_H

#include "lane.h"

#include <iostream>
#include <vector>

struct HighwayMap {
    std::vector<Lane> lanes;

    /**
     *  Default constructor.
     */
    HighwayMap();

    /**
     *  Return the indexes to the lane(s) adjacent to the given one.
     */
    std::vector<size_t> adjacentLanes(size_t lane) const;

    /**
     *  Return the index of the lane closest to the given point.
     */
    size_t closestIndex(double x, double y) const;

    /**
     *  Return the index of the lane closest to the given state.
     */
    size_t closestIndex(const Car &state) const;

    /**
     *  Return the index of the lane closest to the given point.
     */
    size_t closestIndex(double d) const;

    /**
     *  Return a reference to the lane closest to the given point.
     */
    const Lane &closestLane(double x, double y) const;

    /**
     *  Return the number of lanes in the highway.
     */
    size_t size() const;
};

std::istream &operator >> (std::istream &in, HighwayMap &highway);

#endif
