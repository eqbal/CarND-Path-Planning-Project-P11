#ifndef CAR_H
#define CAR_H

#include "waypoints.h"

#include "json.hpp"

struct HighwayMap;

struct Car {
    /** Current position in Cartesian coordinates. */
    double x, y;

    /** Current orientation in radians. */
    double o;

    /** Current position in Frenet coordinates. */
    double s, d;

    /** Current linear speed. */
    double v;

    /** Current lane. */
    size_t lane;

    /**
     *  Default constructor.
     */
    State();

    /**
     *  Update this state with data from the given route and JSON node.
     */
    void update(const HighwayMap &highway, const Waypoints &route, const nlohmann::json &json);

    /**
     *  Convert the given waypoints to a local frame relative to this state.
     */
    void toLocalFrame(Waypoints &waypoints) const;

    /**
     *  Convert the given waypoints to the global frame using this state as reference.
     */
    void toGlobalFrame(Waypoints &waypoints) const;
};

#endif
