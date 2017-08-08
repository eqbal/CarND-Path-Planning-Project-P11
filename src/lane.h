#ifndef LANE_H
#define LANE_H

#include "car.h"

#include <cstddef>
#include <tuple>
#include <vector>

struct Lane: Waypoints {
    double width;

    /** Waypoint positions along the road. */
    std::vector<double> s;

    /** Waypoint orientations along the road. */
    std::vector<double> o;

    /**
     *  Default constructor.
     */
    Lane();

    /**
     *  Create a new Lane of given width.
     */
    Lane(double width);

    /**
     *  Return the index of the lane waypoint closest to the given point.
     */
    size_t closestIndex(double x, double y) const;

    /**
     *  Return the index of the lane waypoint closest to the given point.
     */
    size_t closestIndex(double s) const;

    /**
     *  Return the index of the lane waypoint closest to the given point, and the square distance to it..
     */
    std::tuple<size_t, double> closestIndexD2(double x, double y) const;

    /**
     *  Return the index of the closest waypoint in front of the given pose.
     */
    size_t nextIndex(const Car &state) const;

    /**
     *  Return a list of sample waypoints from the state's location.
     */
    Waypoints sample(const Car &state) const;
};

#endif
