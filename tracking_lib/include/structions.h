#pragma once

#include <iostream>
#include <vector>
#include <iterator>

struct measurement_data {
    double time;
    double cos_v;
    int type;

    measurement_data() : time(0.0), cos_v(0.0), type(0) {}

    measurement_data(double ti, double c, int t)
            : time(ti), cos_v(c), type(t) {}
};

typedef std::vector<measurement_data> peleng_data_storage;
typedef peleng_data_storage::iterator Pel_Iter;

struct filter_params {
    double delta_time;
    double time_last;
    double angle;
    double speed;
    double acceleration;

    filter_params() :
            delta_time(0.0),
            time_last(0.0),
            angle(0.0),
            speed(0.0),
            acceleration(0.0) {}

    filter_params(const filter_params &other) :
            delta_time(other.delta_time),
            time_last(other.time_last),
            angle(other.angle),
            speed(other.speed),
            acceleration(other.acceleration) {}

    void swap(filter_params &first, filter_params &second) {
        using std::swap;
        swap(first.delta_time, second.delta_time);
        swap(first.time_last, second.time_last);
        swap(first.angle, second.angle);
        swap(first.speed, second.speed);
        swap(first.acceleration, second.acceleration);
    }

    filter_params &operator=(const filter_params &other) {
        filter_params temp(other);
        swap(*this, temp);
        return *this;
    }
};
