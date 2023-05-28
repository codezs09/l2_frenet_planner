#ifndef FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#define FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#include <stddef.h>
#include <vector>

using namespace std; 

const size_t MAX_PATH_LENGTH = 100;

struct FrenetInitialConditions {
    double s0;
    double c_speed;
    double c_d;
    double c_d_d;
    double c_d_dd;
    double target_speed;
    vector<double> wx;
    vector<double> wy;
    // int nw;
    vector<double> o_llx;
    vector<double> o_lly;
    vector<double> o_urx;
    vector<double> o_ury;
    // int no;
};

struct FrenetReturnValues {
    int success;
    size_t path_length;
    vector<double> x_path;
    vector<double> y_path;
    vector<double> speeds;
    vector<double> ix;
    vector<double> iy;
    vector<double> iyaw;
    vector<double> d;
    vector<double> s;
    vector<double> speeds_x;
    vector<double> speeds_y;
    vector<double> params;
    vector<double> costs;
};

struct FrenetHyperparameters {
    double max_speed;
    double max_accel;
    double max_curvature;
    double max_road_width_l;
    double max_road_width_r;
    double d_road_w;
    double dt;
    double maxt;
    double mint;
    double d_t_s;
    double n_s_sample;
    double obstacle_clearance;
    double kd;
    double kv;
    double ka;
    double kj;
    double kt;
    double ko;
    double klat;
    double klon;
    int num_threads;
};
#endif //FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
