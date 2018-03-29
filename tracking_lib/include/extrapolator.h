#pragma once

#include "structions.h"

class abstract_extrapolator {

protected:
    bool m_is_extrapolate;

public:
    abstract_extrapolator();

    abstract_extrapolator(bool);

    bool check_extrapolate() const;

    virtual filter_params extrapolate(double) = 0;

    virtual bool put(filter_params &) = 0;

    virtual ~abstract_extrapolator() {};
};

class last_point_extrapolator : public abstract_extrapolator {

public:
    virtual filter_params extrapolate(double);

    virtual bool put(filter_params &);

    virtual ~last_point_extrapolator() {};
private:
    filter_params m_last_data;
};

class linear_extrapolator : public abstract_extrapolator {

public:
    linear_extrapolator();

    virtual filter_params extrapolate(double);

    virtual bool put(filter_params &);

    virtual ~linear_extrapolator() {};
private:
    filter_params m_last_data;

    double calculate_next_extr_angle(double);
};
