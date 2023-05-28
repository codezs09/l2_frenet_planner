#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"
#include "CubicSpline2D.h"
#include "utils.h"

#include <stddef.h>
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace std;


// Compute the frenet optimal trajectory given initial conditions
// in frenet space.
//
// Arguments:
//      fot_ic (FrenetInitialConditions *):
//          struct ptr containing relevant initial conditions to compute
//          Frenet Optimal Trajectory
//      fot_hp (FrenetHyperparameters *):
//          struct ptr containing relevant hyperparameters to compute
//          Frenet Optimal Trajectory
//      x_path, y_path, speeds (double *):
//          ptr to storage arrays for Frenet Optimal Trajectory
//      params (double *):
//          ptr to store initial conditions for debugging
//
// Returns:
//      1 if successful, 0 if failure
//      Also stores the Frenet Optimal Trajectory into x_path, y_path,
//      speeds if it exists
void run_fot(
        FrenetInitialConditions *fot_ic, FrenetHyperparameters *fot_hp,
        FrenetReturnValues *fot_rv
        ) {
    FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(fot_ic, fot_hp);
    FrenetPath* best_frenet_path = fot.getBestPath();
    if (best_frenet_path && !best_frenet_path->x.empty()){
        fot_rv->success = 1;
        int path_length = best_frenet_path->x.size();
        fot_rv->path_length = path_length;
        fot_rv->x_path = best_frenet_path->x;
        fot_rv->y_path = best_frenet_path->y;
        fot_rv->speeds = best_frenet_path->s_d;
        fot_rv->ix = best_frenet_path->ix;
        fot_rv->iy = best_frenet_path->iy;
        fot_rv->iyaw = best_frenet_path->iyaw;
        fot_rv->d = best_frenet_path->d;
        fot_rv->s = best_frenet_path->s;
        fot_rv->speeds_x.resize(path_length);
        fot_rv->speeds_y.resize(path_length);
        for (int i = 0; i < path_length; i++) {
            fot_rv->speeds_x[i] = cos(best_frenet_path->yaw[i]) *
                fot_rv->speeds[i];
            fot_rv->speeds_y[i] = sin(best_frenet_path->yaw[i]) *
                fot_rv->speeds[i];
        }

        // store info for debug
        fot_rv->params.resize(5);
        fot_rv->params[0] = best_frenet_path->s[1];
        fot_rv->params[1] = best_frenet_path->s_d[1];
        fot_rv->params[2] = best_frenet_path->d[1];
        fot_rv->params[3] = best_frenet_path->d_d[1];
        fot_rv->params[4] = best_frenet_path->d_dd[1];

        // store costs for logging
        fot_rv->costs.resize(12);
        fot_rv->costs[0] = best_frenet_path->c_lateral_deviation;
        fot_rv->costs[1] = best_frenet_path->c_lateral_velocity;
        fot_rv->costs[2] = best_frenet_path->c_lateral_acceleration;
        fot_rv->costs[3] = best_frenet_path->c_lateral_jerk;
        fot_rv->costs[4] = best_frenet_path->c_lateral;
        fot_rv->costs[5] = best_frenet_path->c_longitudinal_acceleration;
        fot_rv->costs[6] = best_frenet_path->c_longitudinal_jerk;
        fot_rv->costs[7] = best_frenet_path->c_time_taken;
        fot_rv->costs[8] = best_frenet_path->c_end_speed_deviation;
        fot_rv->costs[9] = best_frenet_path->c_longitudinal;
        fot_rv->costs[10] = best_frenet_path->c_inv_dist_to_obstacles;
        fot_rv->costs[11] = best_frenet_path->cf;
    }
}

// Convert the initial conditions from cartesian space to frenet space
void to_frenet_initial_conditions(
        double s0, double x, double y, double vx,
        double vy, double forward_speed, 
        const vector<double> &xp, const vector<double> &yp,
        vector<double>& initial_conditions
        ) {
    const vector<double>& wx (xp);
    const vector<double>& wy (yp);
    CubicSpline2D* csp = new CubicSpline2D(wx, wy);

    // get distance from car to spline and projection
    double s = csp->find_s(x, y, s0);
    double distance = norm(csp->calc_x(s) - x, csp->calc_y(s) - y);
    tuple<double, double> bvec ((csp->calc_x(s) - x) / distance,
            (csp->calc_y(s) - y) / distance);

    // normal spline vector
    double x0 = csp->calc_x(s0);
    double y0 = csp->calc_y(s0);
    double x1 = csp->calc_x(s0 + 2);
    double y1 = csp->calc_y(s0 + 2);

    // unit vector orthog. to spline
    tuple<double, double> tvec (y1-y0, -(x1-x0));
    as_unit_vector(tvec);

    // compute tangent / normal car vectors
    tuple<double, double> fvec (vx, vy);
    as_unit_vector(fvec);

    // get initial conditions in frenet frame
    initial_conditions.resize(5);
    initial_conditions[0] = s; // current longitudinal position s
    initial_conditions[1] = forward_speed; // speed [m/s]
    // lateral position c_d [m]
    initial_conditions[2] = copysign(distance, dot(tvec, bvec));
    // lateral speed c_d_d [m/s]
    initial_conditions[3] = -forward_speed * dot(tvec, fvec);
    initial_conditions[4] = 0.0; // lateral acceleration c_d_dd [m/s^2]
    // TODO: add lateral acceleration when CARLA 9.7 is patched (IMU)

    delete csp;
}

PYBIND11_MODULE(FotWrapper, m) {
    py::class_<FrenetInitialConditions>(m, "FrenetInitialConditions")
        .def(py::init<>())
        .def_readwrite("s0", &FrenetInitialConditions::s0)
        .def_readwrite("c_speed", &FrenetInitialConditions::c_speed)
        .def_readwrite("c_d", &FrenetInitialConditions::c_d)
        .def_readwrite("c_d_d", &FrenetInitialConditions::c_d_d)
        .def_readwrite("c_d_dd", &FrenetInitialConditions::c_d_dd)
        .def_readwrite("target_speed", &FrenetInitialConditions::target_speed)
        .def_readwrite("wx", &FrenetInitialConditions::wx)
        .def_readwrite("wy", &FrenetInitialConditions::wy)
        .def_readwrite("o_llx", &FrenetInitialConditions::o_llx)
        .def_readwrite("o_lly", &FrenetInitialConditions::o_lly)
        .def_readwrite("o_urx", &FrenetInitialConditions::o_urx)
        .def_readwrite("o_ury", &FrenetInitialConditions::o_ury);
    py::class_<FrenetReturnValues>(m, "FrenetReturnValues")
        .def(py::init<>())
        .def_readwrite("success", &FrenetReturnValues::success)
        .def_readwrite("path_length", &FrenetReturnValues::path_length)
        .def_readwrite("x_path", &FrenetReturnValues::x_path)
        .def_readwrite("y_path", &FrenetReturnValues::y_path)
        .def_readwrite("speeds", &FrenetReturnValues::speeds)
        .def_readwrite("ix", &FrenetReturnValues::ix)
        .def_readwrite("iy", &FrenetReturnValues::iy)
        .def_readwrite("iyaw", &FrenetReturnValues::iyaw)
        .def_readwrite("d", &FrenetReturnValues::d)
        .def_readwrite("s", &FrenetReturnValues::s)
        .def_readwrite("speeds_x", &FrenetReturnValues::speeds_x)
        .def_readwrite("speeds_y", &FrenetReturnValues::speeds_y)
        .def_readwrite("params", &FrenetReturnValues::params)
        .def_readwrite("costs", &FrenetReturnValues::costs);
    py::class_<FrenetHyperparameters>(m, "FrenetHyperparameters")
        .def(py::init<>())
        .def_readwrite("max_speed", &FrenetHyperparameters::max_speed)
        .def_readwrite("max_accel", &FrenetHyperparameters::max_accel)
        .def_readwrite("max_curvature", &FrenetHyperparameters::max_curvature)
        .def_readwrite("max_road_width_l", &FrenetHyperparameters::max_road_width_l)
        .def_readwrite("max_road_width_r", &FrenetHyperparameters::max_road_width_r)
        .def_readwrite("d_road_w", &FrenetHyperparameters::d_road_w)
        .def_readwrite("dt", &FrenetHyperparameters::dt)
        .def_readwrite("maxt", &FrenetHyperparameters::maxt)
        .def_readwrite("mint", &FrenetHyperparameters::mint)
        .def_readwrite("d_t_s", &FrenetHyperparameters::d_t_s)
        .def_readwrite("n_s_sample", &FrenetHyperparameters::n_s_sample)
        .def_readwrite("obstacle_clearance", &FrenetHyperparameters::obstacle_clearance)
        .def_readwrite("kd", &FrenetHyperparameters::kd)
        .def_readwrite("kv", &FrenetHyperparameters::kv)
        .def_readwrite("ka", &FrenetHyperparameters::ka)
        .def_readwrite("kj", &FrenetHyperparameters::kj)
        .def_readwrite("kt", &FrenetHyperparameters::kt)
        .def_readwrite("ko", &FrenetHyperparameters::ko)
        .def_readwrite("klat", &FrenetHyperparameters::klat)
        .def_readwrite("klon", &FrenetHyperparameters::klon)
        .def_readwrite("num_threads", &FrenetHyperparameters::num_threads);

    m.def("run_fot", &run_fot, "Run Frenet Optimal Trajectory");
    m.def("to_frenet_initial_conditions", &to_frenet_initial_conditions,
            "Convert initial conditions from cartesian to frenet space");
}
