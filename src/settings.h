#ifndef SETTINGS_H
#define SETTINGS_H

/** @brief Number of lanes in the highway map. */
#define N_LANES 3

/** @brief Width of each lane in meters. */
#define W_LANE 4.0

/** @brief Number of planned waypoints. */
#define N_PLAN 50

/** @brief Number of planned waypoints to keep between updates. */
#define N_KEEP 20

/** @brief Time gap between waypoints. */
#define T_PLAN 0.02

/** @brief Optimal speed in m/s. */
#define V_PLAN 20.0

/** @brief Order of the polynomial used to fit waypoints. */
#define N_FIT 3

/** @brief Number of sample waypoints extracted from the lane map. */
#define N_SAMPLES 2

/** @brief Maximum longitudinal position along the highway map. */
#define S_MAX 6945.554

#endif
