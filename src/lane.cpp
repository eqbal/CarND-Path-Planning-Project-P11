#include "lane.h"

#include "settings.h"

#include <limits>

Lane::Lane() {
    // Nothing to do.
}

Lane::Lane(double width) {
    this->width = width;
}

inline double distance2(double x_a, double y_a, double x_b, double y_b) {
    double x_d = x_a - x_b;
    double y_d = y_a - y_b;
    return x_d * x_d + y_d * y_d;
}

size_t Lane::closestIndex(double x, double y) const {
    size_t i;
    double _;
    std::tie(i, _) = closestIndexD2(x, y);
    return i;
}

size_t Lane::closestIndex(double s) const {
    double d2_closest = std::numeric_limits<double>::max();
    size_t i_closest = 0;

    for(size_t i = 0, n = size(); i < n; i++) {
        double d2 = pow(s - this->s[i], 2.0);
        if(d2 < d2_closest) {
            d2_closest = d2;
            i_closest = i;
        }
    }

    return i_closest;
}

std::tuple<size_t, double> Lane::closestIndexD2(double x, double y) const {
    double d2_closest = std::numeric_limits<double>::max();
    size_t i_closest = 0;

    for(size_t i = 0, n = size(); i < n; i++) {
        double d2 = distance2(x, y, this->x[i], this->y[i]);
        if(d2 < d2_closest) {
            d2_closest = d2;
            i_closest = i;
        }
    }

    return std::make_tuple(i_closest, d2_closest);
}

size_t Lane::nextIndex(const Car &state) const {
    static double PI_025 = 0.25 * M_PI;

    double x_0 = state.x;
    double y_0 = state.y;
    double o_0 = state.o;

    size_t i = closestIndex(x_0, y_0);

    double x_1 = x[i];
    double y_1 = y[i];
    double o_1 = atan2(y_1 - y_0, x_1 - x_0);

    double o_d = std::abs(o_1 - o_0);
    if(o_d > PI_025) {
        i = (i + 1) % size();
    }

    return i;
}

Waypoints Lane::sample(const Car &state) const {
    std::vector<double> x = {state.x};
    std::vector<double> y = {state.y};

    size_t n = size();
    size_t k = nextIndex(state);

    // If the next waypoint is closer than a half second away, start at the one after that.
    if (distance2(state.x, state.y, this->x[k], this->y[k]) < 0.25 * state.v * state.v) {
        k++;
    }

    for(size_t i = 0; i < N_SAMPLES; ++i, ++k) {
        x.push_back(this->x[k % n]);
        y.push_back(this->y[k % n]);
    }

    return {x, y};
}
