#include "extrapolator.h"
#include "structions.h"
#include <cmath>
#include <iostream>

#define _DEBUG_EXTRAPOLATOR 0

abstract_extrapolator::abstract_extrapolator() : m_is_extrapolate(false) {}

bool abstract_extrapolator::check_extrapolate() const {
    return m_is_extrapolate;
}

bool last_point_extrapolator::put(filter_params &a_data) {
    m_last_data = a_data;
    m_is_extrapolate = true;

    return true;
}

filter_params last_point_extrapolator::extrapolate(double a_time) {
    if (!m_is_extrapolate) {
        throw std::runtime_error("last_point_extrapolator::extrapolate error");
    }

#if _DEBUG_EXTRAPOLATOR
    std::cout << "Extrapolator:\n" << m_last_data.angle << "\t"
            << m_last_data.time_last << "\t" << m_last_data.speed << std::endl;
#endif

    return m_last_data;
}

/**********************************/
linear_extrapolator::linear_extrapolator() {}

bool linear_extrapolator::put(filter_params &a_data) {
    m_last_data = a_data;
    return true;
}

double linear_extrapolator::calculate_next_extr_angle(double a_time) {
#if _DEBUG_EXTRAPOLATOR
    std::cout << "Extrapolator:\n" << m_last_data.angle << "\t"
            << m_last_data.time_last << "\t" << m_last_data.speed << std::endl;
    std::cout << "Current time: " << a_time << std::endl;
#endif
    return m_last_data.angle + m_last_data.speed * (a_time - m_last_data.time_last);
}

filter_params linear_extrapolator::extrapolate(double a_time) {
    filter_params temp;

    temp.angle = calculate_next_extr_angle(a_time);
    temp.time_last = a_time;

    return temp;
}
