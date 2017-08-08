#include "highway_map.h"

#include "settings.h"

#include <cmath>
#include <limits>
#include <tuple>

HighwayMap::HighwayMap():
    lanes(N_LANES, Lane(W_LANE))
{
    // Nothing to do.
}

std::vector<size_t> HighwayMap::adjacentLanes(size_t lane) const {
    if (lane == 0) {
        return {1};
    }

    size_t l = lanes.size() - 1;
    if (lane >= l) {
        return {l - 1};
    }

    return {lane - 1, lane + 1};
}

size_t HighwayMap::closestIndex(double x, double y) const {
    size_t i_closest = 0;
    double d2_closest = std::numeric_limits<double>::max();

    for(size_t i = 0, n = size(); i < n; i++) {
        size_t _;
        double d2;
        std::tie(_, d2) = lanes[i].closestIndexD2(x, y);
        if (d2 < d2_closest) {
            d2_closest = d2;
            i_closest = i;
        }
    }

    return i_closest;
}

size_t HighwayMap::closestIndex(const State &state) const {
    return closestIndex(state.x, state.y);
}

size_t HighwayMap::closestIndex(double d) const {
    size_t i_closest = 0;
    double d2_closest = std::numeric_limits<double>::max();

    for(size_t i = 0, n = size(); i < n; i++) {
        double d2 = pow(d - (i + 0.5) * W_LANE, 2.0);
        if (d2 < d2_closest) {
            d2_closest = d2;
            i_closest = i;
        }
    }

    return i_closest;
}

const Lane &HighwayMap::closestLane(double x, double y) const {
    return lanes[closestIndex(x, y)];
}

size_t HighwayMap::size() const {
    return lanes.size();
}

std::istream &operator >> (std::istream &in, HighwayMap &highway) {
    // Read waypoints located over the line separating highway sides.
    std::vector<double> line_x;
    std::vector<double> line_y;
    std::vector<double> line_s;
    for (;;) {
        double x, y, s, d_x, d_y;
        in >> x >> y >> s >> d_x >> d_y;
        if (in.eof()) {
            break;
        }

        line_x.push_back(x);
        line_y.push_back(y);
        line_s.push_back(s);
    }

    // Compute waypoints for each highway lane.
    for (size_t i = 0, m = line_x.size(); i < m; ++i) {
        // Fetch the current longitudinal position.
        double s = line_s[i];

        // Fetch the "current" and "next" waypoints.
        double x_0 = line_x[i];
        double y_0 = line_y[i];
        double x_1 = line_x[(i + 1) % m];
        double y_1 = line_y[(i + 1) % m];

        // Compute a normalized vector perpendicular to the
        // direction from (x_0, y_0) to (x_1, y_1)
        // and pointing rightwards.
        double x_d = x_1 - x_0;
        double y_d = y_1 - y_0;
        double l_d = std::sqrt(x_d * x_d + y_d * y_d);
        double x_p = y_d / l_d;
        double y_p = -x_d / l_d;

        // Compute the waypoints for each lane in the highway map.
        std::vector<Lane> &lanes = highway.lanes;
        for (size_t j = 0, n = lanes.size(); j < n; ++j) {
            Lane &lane = lanes[j];
            double d = (j + 0.5) * lane.width;
            lane.x.push_back(x_0 + x_p * d);
            lane.y.push_back(y_0 + y_p * d);
            lane.s.push_back(s);
            lane.o.push_back(std::atan2(y_d, x_d));
        }
    }

    return in;
}
