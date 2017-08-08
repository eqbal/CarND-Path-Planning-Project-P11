#include "car.h"
#include "highway_map.h"
#include "settings.h"

#include <cmath>

Car::State():
    x(0),
    y(0),
    o(0),
    s(0),
    d(0),
    v(0)
{
}

void Car::update(const HighwayMap &highway, const Waypoints &route, const nlohmann::json &json) {
    x = json["x"];
    y = json["y"];
    s = json["s"];
    d = json["d"];
    o = ((double) json["yaw"]) * M_PI / 180.0; // Convert from degrees to radians
    v = ((double) json["speed"]) * 0.447; // Convert from MPH to m/s

    size_t n = route.size();
    if (n == 0) {
        lane = highway.closestIndex(d);
        return;
    }

    // If the route has at least one waypoint, use it to
    // update the state's position.

    size_t l = n - 1;
    double x_a = x;
    double y_a = y;
    if (l > 0) {
        x_a = route.x[l - 1];
        y_a = route.y[l - 1];
    }

    double x_b = route.x[l];
    double y_b = route.y[l];
    double x_d = x_b - x_a;
    double y_d = y_b - y_a;
    double e = std::sqrt(x_d * x_d + y_d * y_d);

    x = x_b;
    y = y_b;
    o = std::atan2(y_d, x_d);
    v = e / T_PLAN;
    lane = highway.closestIndex(x, y);

    const Lane &closest = highway.lanes[lane];
    double w = o - closest.o[closest.closestIndex(x, y)];
    s += e * std::cos(w);
    d -= e * std::sin(w);
}

void Car::toLocalFrame(Waypoints &waypoints) const {
    double cos_o = std::cos(o);
    double sin_o = std::sin(o);

    for (int i = 0, n = waypoints.size(); i < n; ++i) {
        double &x_i = waypoints.x[i];
        double &y_i = waypoints.y[i];

        double x_s = x_i - x;
        double y_s = y_i - y;

        x_i = x_s * cos_o + y_s * sin_o;
        y_i = y_s * cos_o - x_s * sin_o;
    }
}

void Car::toGlobalFrame(Waypoints &waypoints) const {
    double cos_o = std::cos(o);
    double sin_o = std::sin(o);

    for (int i = 0, n = waypoints.size(); i < n; ++i) {
        double &x_i = waypoints.x[i];
        double &y_i = waypoints.y[i];

        double x_r = x_i * cos_o - y_i * sin_o;
        double y_r = y_i * cos_o + x_i * sin_o;

        x_i = x + x_r;
        y_i = y + y_r;
    }
}
