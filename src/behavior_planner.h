#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "highway_map.h"
#include "obstacles.h"
#include "car.h"

#include "json.hpp"

#include <Eigen/Dense>

#include <functional>

struct BehaviorPlanner {
    /**
     *  A state in the Behavior Planner's state machine.
     */
    struct Behavior: std::function<Behavior(BehaviorPlanner&)> {
        /**  A convenient alias for the base type. */
        typedef std::function<Behavior(BehaviorPlanner&)> functor;

        /**
         *  Default constructor.
         */
        Behavior():
            functor()
        {
            // Nothing to do.
        }

        /**
         *  Wraps a custom function into a Behavior object.
         */
        template<class F> Behavior(F f):
            functor(f)
        {
            // Nothing to do.
        }
    };

    /**  Currently selected behavior. */
    Behavior behavior;

    /**  Map of the highway on which the navigator will drive. */
    HighwayMap highway;

    /**  Current lane, or departing lane if the car is in the process of changing lanes. */
    size_t origin_lane;

    /**  Current lane, or approaching lane if the car is in the process of changing lanes. */
    size_t target_lane;

    /**  Current car position and speed. */
    Car state;

    /**  Surrounding obstacles. */
    Obstacles obstacles;

    /**  Target speed. */
    double v;

    /**  Target route. */
    Eigen::VectorXd route;

    /**
     *  Default constructor.
     */
    BehaviorPlanner();

    /**
     *  Update the behavior planner with data from the currently planned route and given JSON node.
     */
    void update(const Waypoints &plan, const nlohmann::json &json);
};

#endif
