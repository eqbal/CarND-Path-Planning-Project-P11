#include "path_planner.h"

#include "settings.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

#include <Eigen/Dense>

using Eigen::VectorXd;

static const size_t X = 0;
static const size_t Y = 1;
static const size_t A = 1;
static const size_t D = 2;
static const size_t SIZEOF_POINT = 2;
static const size_t SIZEOF_CONSTRAINT = 3;

// Options for IPOPT solver.
static const std::string options =
    "Integer print_level 0\n"
    "Sparse true reverse\n"
    "Numeric max_cpu_time 0.2\n";



struct Cost {
    /** @brief Basic scalar value type. */
    typedef AD<double> Scalar;

    /** @brief Differentiable variable vector type. */
    typedef CPPAD_TESTVECTOR(Scalar) ADvector;

    /** @brief Number of `(x, y)` points in the plan. */
    size_t n_plan;

    /** @brief Previous longitudinal speed. */
    Scalar v_0;

    /** @brief Reference longitudinal speed. */
    Scalar v_r;

    /** @brief Coefficients of the polynomial describing the reference route. */
    VectorXd route;

    /**
     * @brief Create a new optimization task with given initial speed and reference route.
     */
    Cost(size_t n_plan, double v_0, double v_r, const VectorXd &route) {
        this->n_plan = n_plan;
        this->v_0 = v_0;
        this->v_r = v_r;
        this->route = route;
    }

    /**
     * @brief Compute the cost function for the OPP.
     */
    void operator () (ADvector &fg, const ADvector &vars) {
        Scalar w_0 = 0;

        for (size_t i = 0, n = n_plan - 1; i < n; ++i) {
            auto &x_0 = vars[X + SIZEOF_POINT * i];
            auto &y_0 = vars[Y + SIZEOF_POINT * i];
            auto &x_1 = vars[X + SIZEOF_POINT * (i + 1)];
            auto &y_1 = vars[Y + SIZEOF_POINT * (i + 1)];

            auto x_d = x_1 - x_0;
            auto y_d = y_1 - y_0;

            // Longitudinal and lateral speeds.
            auto v_1 = x_d / T_PLAN;
            auto w_1 = y_d / T_PLAN;

            // Longitudinal and lateral accelerations.
            auto a_1 = (v_1 - v_0) / T_PLAN;
            auto d_1 = (w_1 - w_0) / T_PLAN;

            // Reference state.
            auto y_r = reference(x_1);

            // Contributions to the cost function.
            fg[0] += CppAD::pow(a_1, 2);
            fg[0] += 0.0001 * CppAD::pow(d_1, 2);
            fg[0] += CppAD::pow(y_r - y_1, 2);
            fg[0] += CppAD::pow(v_r - v_1, 2);

            // Constraint functions values.
            fg[1 + X + SIZEOF_CONSTRAINT * i] = x_d;
            fg[1 + A + SIZEOF_CONSTRAINT * i] = a_1;
            fg[1 + D + SIZEOF_CONSTRAINT * i] = d_1;

            v_0 = v_1;
            w_0 = w_1;
        }
    }

private:
    /**
     * @brief Compute the `y` coordinate for the reference route.
     */
    Scalar reference(const Scalar &x) const {
        Scalar y = route(0);
        for (size_t i = 1, n = route.rows(); i < n; ++i) {
            y += route(i) * CppAD::pow(x, i);
        }

        return y;
    }
};

const Waypoints &PathPlanner::operator () (const BehaviorPlanner &behavior) {
    // Differentiable value vector type.
    typedef CPPAD_TESTVECTOR(double) Vector;

    // Initialize independent variable and bounds vectors.
    const Car &state = behavior.state;
    size_t n_plan = N_PLAN - plan.size();
    size_t n_vars = n_plan * SIZEOF_POINT;
    Vector vars(n_vars);
    Vector vars_lowerbound(n_vars);
    Vector vars_upperbound(n_vars);
    for (size_t i = 0; i < n_vars; i += SIZEOF_POINT) {
        size_t i_x = i + X;
        size_t i_y = i + Y;

        vars[i_x] = 0;
        vars[i_y] = 0;

        vars_lowerbound[i_x] = 0;
        vars_upperbound[i_x] = V_PLAN * T_PLAN * n_plan;

        vars_lowerbound[i_y] = state.d - W_LANE * N_LANES;
        vars_upperbound[i_y] = state.d;
    }

    // Lock the first point in place, as it represents the car's current position.
    vars_lowerbound[X] = 0;
    vars_upperbound[X] = 0;
    vars_lowerbound[Y] = 0;
    vars_upperbound[Y] = 0;

    // Initialize constraint vectors.
    size_t n_constraints = (n_plan - 1) * SIZEOF_CONSTRAINT;
    Vector constraints_lowerbound(n_constraints);
    Vector constraints_upperbound(n_constraints);
    for (size_t i = 0; i < n_constraints; i += SIZEOF_CONSTRAINT) {
        // Ensure x(t) is a strictly increasing function.
        constraints_lowerbound[i + X] = 0.01;
        constraints_upperbound[i + X] = 1.1 * V_PLAN * T_PLAN;

        // Ensure longitudinal acceleration stays within reasonable limits.
        constraints_lowerbound[i + A] = -4.0;
        constraints_upperbound[i + A] = 4.0;

        // Ensure lateral acceleration stays within reasonable limits.
        constraints_lowerbound[i + D] = -4.0;
        constraints_upperbound[i + D] = 4.0;
    }

    // Define the cost function.
    Cost cost(n_plan, state.v, behavior.v, behavior.route);

    // Solution to the cost optimization problem.
    CppAD::ipopt::solve_result<Vector> solution;

    // Call the solver on the cost function and given parameters.
    CppAD::ipopt::solve<Vector, Cost>(
        options,
        vars,
        vars_lowerbound,
        vars_upperbound,
        constraints_lowerbound,
        constraints_upperbound,
        cost,
        solution
    );

    auto &control = solution.x;
//     auto value = solution.obj_value;
//     auto status = (solution.status == CppAD::ipopt::solve_result<Vector>::success ? "succeeded" : "failed");
//     std::cout << "Solver " << status << ", final cost value = " << value << std::endl;

    Waypoints append;
    std::vector<double> &x = append.x;
    std::vector<double> &y = append.y;

    // Discard first waypoint, which is the same as the last waypoint in the
    // current route.
    for (size_t i = SIZEOF_POINT; i < n_vars; i += SIZEOF_POINT) {
        x.push_back(control[i + X]);
        y.push_back(control[i + Y]);
    }

    state.toGlobalFrame(append);
    plan.x.insert(plan.x.end(), append.x.begin(), append.x.end());
    plan.y.insert(plan.y.end(), append.y.begin(), append.y.end());

    return plan;
}
