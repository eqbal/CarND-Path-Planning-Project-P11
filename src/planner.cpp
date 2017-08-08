#include "planner.h"

const Waypoints &Planner::operator () (const nlohmann::json &json) {
    path.plan.update(json);
    behavior.update(path.plan, json);
    return path(behavior);
}
