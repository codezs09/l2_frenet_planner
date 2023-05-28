#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"

#include <iostream>
#include <vector>

using namespace std;

int main() {
    vector<double> wx = {132.67, 128.67, 124.67, 120.67, 116.67, 112.67, 108.67,
                   104.67, 101.43,  97.77,  94.84,  92.89,  92.4 ,  92.4 ,
                   92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.39,  92.39,
                   92.39,  92.39,  92.39,  92.39};
    vector<double> wy = {195.14, 195.14, 195.14, 195.14, 195.14, 195.14, 195.14,
                   195.14, 195.14, 195.03, 193.88, 191.75, 188.72, 185.32,
                   181.32, 177.32, 173.32, 169.32, 165.32, 161.32, 157.32,
                   153.32, 149.32, 145.32, 141.84};
    vector<double> o_llx = {92.89};
    vector<double> o_lly = {191.75};
    vector<double> o_urx = {92.89};
    vector<double> o_ury = {191.75};

    // set up experiment
    FrenetInitialConditions fot_ic = {
        34.6,
        7.10964962,
        -1.35277168,
        -1.86,
        0.0,
        10,
        wx,
        wy,
        o_llx,
        o_lly,
        o_urx,
        o_ury
    };
    FrenetHyperparameters fot_hp = {
        25.0,
        15.0,
        15.0,
        5.0,
        5.0,
        0.5,
        0.2,
        5.0,
        2.0,
        0.5,
        2.0,
        0.1,
        1.0,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        1.0,
        1.0,
        2 // num thread
    };

    // run experiment
    FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp);
    FrenetPath* best_frenet_path = fot.getBestPath();
    if (best_frenet_path) {
        cout << "Success\n";
        return 1;
    }
    cout << "Failure\n";
    return 0;
}