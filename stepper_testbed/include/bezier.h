#pragma once
#include <Arduino.h>
#include <vector>
#include <tuple>

const int bez_res = 200;
const float anchor1_x = 0.5;
const float anchor2_x = 0.5;
using namespace std;
tuple<int, int> anchor_2(0.5, 1);

vector <float> lookup_table(bez_res, 0.0);
bool bezier_populated = false;
float bezcel_dist = 0.0;

float _cubicBezier1D(float t, float p0, float p1, float p2, float p3) {
    float u = 1.0-t;
    return static_cast<float>(u*u*u*p0 + 3.0*u*u*t*p1 + 3.0*u*t*t*p2 + t*t*t*p3);
}

void populateBezier() {
    std::vector<float> t_to_x(bez_res);

    //finding nonlinear x from t-progression; fast > slow > fast
    //(bez_res) datapoints, each from 0 to 1
    for (int i=0; i<bez_res; i++) {
        float n = ((float)i/(float)bez_res);
        t_to_x[i] = _cubicBezier1D(n, 
            0.0, 
            anchor1_x, 
            anchor2_x,
            1.0);
    }
    //finding associated t for constant x-progression over (bez_res) datapoints
    for (int i=0; i<bez_res; i++) {
        //instantiate pointers from 0 to bez_res-1
        int lo = 0;
        int hi = (bez_res-1);
        
        //goal goes from 0 to 1
        float goal = (float)i / (float)(bez_res-1);
        while (hi - lo > 1) {
            int mid = (lo + hi)/2;
            if (t_to_x[mid] < goal) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        //lo, hi: lower and upper bounds on t from 0 to bez_res - 1;
        float frac = 0.0;
        if (t_to_x[hi] > t_to_x[lo]) {
            frac = (((float)goal - t_to_x[lo])/(t_to_x[hi]-t_to_x[lo]));
        }

        float t0 = static_cast<float>((float)lo/(float)(bez_res-1));
        float t1 = static_cast<float>((float)hi/(float)(bez_res-1));
        float interp_t = t0 + (float)(frac*(t1-t0));
        //interp_t: the more exact time that each linearized x-value is hit, from 0 to 1;

        float y_val = static_cast<float>((3*(1-interp_t)*interp_t*interp_t + interp_t*interp_t*interp_t));
        if (y_val > 0.99) y_val = 1.0;
        lookup_table.at(i) = y_val;
    }
}

float bezcelDist() {
    return bezcel_dist;
}

void calculate_bezcelDist(float maxSpeed) {
    float total_steps = 0.0;
    for (int i=0; i<(int)(lookup_table.size()-1); i++) {
        float steprate1 = maxSpeed*lookup_table[i];
        float steprate2 = maxSpeed*lookup_table[i+1];
        float trapezoid_slice = (0.5 * (steprate1+steprate2))*(1.0/200.0);
        // Serial.println(trapezoid_slice);
        total_steps += trapezoid_slice;
    }
    bezcel_dist = total_steps;
}

vector<float> lookup(float maxSpeed) {
    while (!Serial) {;}
    if (!bezier_populated) {populateBezier();}
    if (bezcel_dist == 0) {calculate_bezcelDist(maxSpeed);}
    // ***************** VERIFY BEZIER CURVE FOR DEBUGGING *******************
    // Serial.printf("%s BEZIER VALUES CALCULATED:\n", bez_res);
    // for (int i=0; i<_lookup_table.size(); i++) {
    //     Serial.println(_lookup_table[i]);
    // }
    // ***************** VERIFY RIEMANN SUM FOR DEBUGGING ********************
    // Serial.printf("TRAPEZOIDAL RIEMANN SUM/BEZCEL: %.6f", bezcel_dist);
    return lookup_table;
}