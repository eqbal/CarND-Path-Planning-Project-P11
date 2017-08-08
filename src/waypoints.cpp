#include "waypoints.h"

#include "settings.h"

#include <Eigen/QR>

using Eigen::ArrayXd;
using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <cmath>

Waypoints::Waypoints() {
    // Nothing to so.
}

Waypoints::Waypoints(const std::vector<double> &x, const std::vector<double> &y) {
    this->x = x;
    this->y = y;
}

VectorXd Waypoints::fit() const {
    int rows = size();
    Map<const ArrayXd> x(this->x.data(), rows);
    Map<const VectorXd> y(this->y.data(), rows);

    MatrixXd A(rows, N_FIT + 1);
    A.block(0, 0, rows, 1).fill(1.0);
    for (int j = 0; j < N_FIT; j++) {
        auto Aj = A.block(0, j, rows, 1).array();
        A.block(0, j + 1, rows, 1) = Aj * x;
    }

    auto Q = A.householderQr();
    auto result = Q.solve(y);
    return result;
}

void Waypoints::update(const nlohmann::json &json) {
    *this = {json["previous_path_x"], json["previous_path_y"]};

    if (size() > N_KEEP) {
        x.erase(x.begin() + N_KEEP, x.end());
        y.erase(y.begin() + N_KEEP, y.end());
    }
}

size_t Waypoints::size() const {
    return x.size();
}

double eval(const VectorXd &a, double x) {
    double y = a(0);
    for (int i = 1, n = a.rows(); i < n; ++i) {
        y += a(i) * std::pow(x, i);
    }

    return y;
}
