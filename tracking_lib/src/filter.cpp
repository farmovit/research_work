#include "filter.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <cfloat>
#include <sstream>

#include <unsupported/Eigen/MatrixFunctions>

#define _DEBUG_FILTRATION 0


abstract_filter::abstract_filter(bool flag = false) : m_is_initialize(flag) {}

bool abstract_filter::check_state() const {
    return m_is_initialize;
}

bool abstract_filter::check_time(const measurement_data &next_data) {
    const double allowed_time_res = 5e-5; // seconds
    return fabs(next_data.time - m_filtrated_data.time_last) > allowed_time_res;
}

filter_params abstract_filter::get_filtrated_data() {
    return m_filtrated_data;
}

/*----------------------------- ALPHA-BETA ---------------------------------*/

alpha_beta_filter::alpha_beta_filter(const double &a_alpha, const double &a_beta) :
        m_alpha(a_alpha), m_beta(a_beta) {}

void alpha_beta_filter::initialize(const filter_params &from) {
#ifdef _DEBUG_ALPHA
    std::cout << "Initializing filter" << std::endl;
#endif

    m_filtrated_data = from;
    m_is_initialize = true;
}

bool alpha_beta_filter::filtrate(const measurement_data &next_data) {
    if (!check_time(next_data)) {
        return false;
    }

    double delta_time = fabs(next_data.time - m_filtrated_data.time_last);

    double extrapolated_angle = m_filtrated_data.angle +
                                delta_time * m_filtrated_data.speed;
    double extrapolated_speed = m_filtrated_data.speed;

#if DEBUG_FILTRATION
    std::cout << "Extrapolated angle: " << extrapolated_angle << std::endl;
    std::cout << "Extrapolated speed: " << extrapolated_speed << std::endl;
#endif

    double residual = next_data.cos_v - extrapolated_angle;
    double corrected_angle = extrapolated_angle + m_alpha * residual;
    double corrected_speed = extrapolated_speed + (m_beta * residual) / delta_time;

#if DEBUG_FILTRATION
    std::cout << "Corrected angle: " << corrected_angle << std::endl;
    std::cout << "Corrected speed: " << corrected_speed << std::endl;
#endif

    m_filtrated_data.angle = corrected_angle;
    m_filtrated_data.speed = corrected_speed;
    m_filtrated_data.time_last = next_data.time;

    return true;
}

/*----------------------------- ALPHA-BETA-GAMMA ---------------------------*/

alpha_beta_gamma_filter::alpha_beta_gamma_filter(
        const double &a_alpha,
        const double &a_beta,
        const double &a_gamma)
        :
        m_alpha(a_alpha),
        m_beta(a_beta),
        m_gamma(a_gamma) {
}

void alpha_beta_gamma_filter::initialize(const filter_params &from) {
    m_filtrated_data = from;
    m_is_initialize = true;
}

bool alpha_beta_gamma_filter::filtrate(const measurement_data &next_data) {
    if (!check_time(next_data)) {
        return false;
    }

    double delta_time = fabs(next_data.time - m_filtrated_data.time_last);

    double extrapolated_angle =
            m_filtrated_data.angle + delta_time * m_filtrated_data.speed +
            0.5 * std::pow(delta_time, 2) * m_filtrated_data.acceleration;
    double extrapolated_speed = m_filtrated_data.speed +
                                delta_time * m_filtrated_data.acceleration;
    double extrapolated_acceleration = m_filtrated_data.acceleration;

    double residual = next_data.cos_v - extrapolated_angle;

    double corrected_angle = extrapolated_angle + m_alpha * residual;
    double corrected_speed = extrapolated_speed + (m_beta * residual) / delta_time;
    double corrected_acceleration = extrapolated_acceleration +
                                    (m_gamma * residual) / std::pow(delta_time, 2);

    m_filtrated_data.angle = corrected_angle;
    m_filtrated_data.speed = corrected_speed;
    m_filtrated_data.acceleration = corrected_acceleration;
    m_filtrated_data.time_last = next_data.time;

    return true;
}


/*----------------------------- STATIC KALMAN -----------------------------*/

static_kalman_filter::static_kalman_filter(
        const Eigen::MatrixXd &a_H,
        const Eigen::MatrixXd &a_R,
        const Eigen::MatrixXd &a_F,
        const Eigen::MatrixXd &a_Q,
        const Eigen::MatrixXd &a_P,
        const Eigen::VectorXd &a_Xk_)
        :
        m_H(a_H),
        m_R(a_R),
        m_F(a_F),
        m_Q(a_Q),
        m_P(a_P),
        m_Xk_(a_Xk_) {}

void static_kalman_filter::initialize(const filter_params &from) {
    set_Xk_(from.angle, from.speed);

#if _DEBUG_FILTRATION
    PrintP();
    PrintF();
    PrintH();
    PrintQ();
    PrintR();
    PrintXk_();

  std::cout << "INFO: ---------------------------------" << std::endl;
#endif

    m_filtrated_data = from;
    m_is_initialize = true;
}

bool static_kalman_filter::filtrate(const measurement_data &next_data) {
    if (!check_time(next_data)) {
        return false;
    }

    Eigen::VectorXd measurement(1);
    measurement << next_data.cos_v;

    Eigen::MatrixXd unit_matrix(2, 2);
    unit_matrix << 1, 0, 0, 1;

#if _DEBUG_FILTRATION
    std::cout << "-----------KALMAN-----------" << std::endl;
    std::cout << "KALMAN: Current matrix: " << std::endl;
    PrintXk_();
    PrintP();
    PrintQ();
    PrintH();
    PrintR();
    std::cout << "KALMAN: Input measurement = " << measurement << std::endl;
#endif

    double delta_time = fabs(next_data.time - m_filtrated_data.time_last);
    set_F(delta_time);

#if _DEBUG_FILTRATION
    std::cout << "KALMAN: delta_time = " << delta_time << std::endl;
    PrintF();
#endif

    Eigen::MatrixXd extrapolated_obs = m_F * m_Xk_;
#if _DEBUG_FILTRATION
    std::cout << "KALMAN: extrapolated_obs = prod(m_F, m_Xk_) = " << extrapolated_obs << std::endl;
#endif

    Eigen::MatrixXd tmp = m_P * m_F.transpose();

#if _DEBUG_FILTRATION
    std::cout << "KALMAN: tmp = m_P * m_F.transpose() = " << tmp << std::endl;
#endif

    m_P = m_F * tmp + m_Q;

#if _DEBUG_FILTRATION
    std::cout << "KALMAN: m_P = m_F * tmp + m_Q = " << m_P << std::endl;
#endif

    tmp = m_P * m_H.transpose();

#if _DEBUG_FILTRATION
    std::cout << "KALMAN: tmp = m_P * m_H.transpose() = " << tmp << std::endl;
#endif

    Eigen::MatrixXd innovation_covariance = m_H * tmp + m_R;

#if _DEBUG_FILTRATION
    std::cout << "KALMAN: innovation_covariance = m_H * tmp + m_R = " << innovation_covariance << std::endl;
#endif

    tmp = m_P * m_H.transpose();

#if _DEBUG_FILTRATION
    std::cout << "KALMAN: tmp = m_P * m_H.transpose() = " << tmp << std::endl;
#endif

    Eigen::MatrixXd kalman_gain = tmp * innovation_covariance.inverse();

#if _DEBUG_FILTRATION
    std::cout << "KALMAN: kalman_gain = prod(tmp, innovation_covariance_inv) = " << kalman_gain << std::endl;
#endif

    tmp = measurement - m_H * extrapolated_obs;

#if _DEBUG_FILTRATION
    std::cout << "KALMAN: tmp = measurement - m_H * extrapolated_obs = " << tmp << std::endl;
#endif

    Eigen::MatrixXd corrected_obs = extrapolated_obs + kalman_gain * tmp;

#if _DEBUG_FILTRATION
    std::cout << "KALMAN:corrected_obs = extrapolated_obs + kalman_gain * tmp = " << corrected_obs << std::endl;
#endif

    tmp = unit_matrix - kalman_gain * m_H;

#if _DEBUG_FILTRATION
    std::cout << "KALMAN: tmp = unit_matrix - kalman_gain * m_H = " << tmp << std::endl;
#endif

    m_P = tmp * m_P;

#if _DEBUG_FILTRATION
    std::cout << "KALMAN: m_P = tmp * m_P = " << m_P << std::endl;
#endif

    set_Xk_(corrected_obs);

    m_filtrated_data.angle = corrected_obs(0, 0);
    m_filtrated_data.speed = corrected_obs(1, 0);
    m_filtrated_data.time_last = next_data.time;

#if _DEBUG_FILTRATION
    std::cout << std::fixed << std::setprecision(5) << std::setw(5);
    std::cout << "KALMAN: Result:\n";
    std::cout << "Time\tangle\tspeed\t\n";
    std::cout << m_filtrated_data.time_last << "\t" << m_filtrated_data.angle << "\t" << m_filtrated_data.speed << std::endl;
    std::cout << "---------------------------------" << std::endl;
#endif

    return true;
}

void static_kalman_filter::set_R(const Eigen::MatrixXd &a_R) {
    m_R = a_R;
}

void static_kalman_filter::set_H(const Eigen::MatrixXd &a_H) {
    m_H = a_H;
}

void static_kalman_filter::set_F(const Eigen::MatrixXd &a_F) {
    m_F = a_F;
}

void static_kalman_filter::set_F(const double &a_dtime) {
    m_F << 1, a_dtime, 0, 1;
}

void static_kalman_filter::set_P(const Eigen::MatrixXd &a_P) {
    m_P = a_P;
}

void static_kalman_filter::set_Q(const Eigen::MatrixXd &a_Q) {
    m_Q = a_Q;
}

void static_kalman_filter::set_Xk_(const double &angle, const double &speed) {
    m_Xk_ << angle, speed;
}

void static_kalman_filter::set_Xk_(const Eigen::VectorXd &a_Xk_) {
    m_Xk_ = a_Xk_;
}

void static_kalman_filter::PrintF() {
    std::cout << "KALMAN: m_F = " << m_F << std::endl;
}

void static_kalman_filter::PrintH() {
    std::cout << "KALMAN: m_H = " << m_H << std::endl;
}

void static_kalman_filter::PrintQ() {
    std::cout << "KALMAN: m_Q = " << m_Q << std::endl;
}

void static_kalman_filter::PrintR() {
    std::cout << "KALMAN: m_R = " << m_R << std::endl;
}

void static_kalman_filter::PrintP() {
    std::cout << "KALMAN: m_P = " << m_P << std::endl;
}

void static_kalman_filter::PrintXk_() {
    std::cout << "KALMAN: m_Xk_ = " << m_Xk_ << std::endl;
}

/*----------------------------- DYNAMIC KALMAN -----------------------------*/

dynamic_kalman_filter::dynamic_kalman_filter(
        const Eigen::MatrixXd &a_H,
        const Eigen::MatrixXd &a_R,
        const Eigen::MatrixXd &a_F,
        const Eigen::MatrixXd &a_Q,
        const Eigen::MatrixXd &a_P,
        const Eigen::VectorXd &a_Xk_,
        const double &a_velocity)
        :
        m_static_kalman(a_H, a_R, a_F, a_Q, a_P, a_Xk_),
        m_velocity(a_velocity) {}

dynamic_kalman_filter::dynamic_kalman_filter(
        const static_kalman_filter &a_kalman_filter,
        const double &a_velocity)
        :
        m_static_kalman(a_kalman_filter),
        m_velocity(a_velocity) {}

void dynamic_kalman_filter::initialize(const filter_params &from) {
    m_filtrated_data = from;
    m_is_initialize = true;
    m_static_kalman.initialize(from);
}

bool dynamic_kalman_filter::filtrate(const measurement_data &a_data) {
    m_static_kalman.set_Q(get_Q(get_distance(a_data)));

    if (!m_static_kalman.filtrate(a_data)) {
        return false;
    }

    m_filtrated_data = m_static_kalman.get_filtrated_data();

    return true;
}

double dynamic_kalman_filter::get_distance(const measurement_data &a_data) {
    double distance = -1 * m_velocity * std::pow(std::sqrt(1 - std::pow(a_data.cos_v, 2)), 3) /
                      ((a_data.cos_v - m_filtrated_data.angle) / (a_data.time - m_filtrated_data.time_last));

    if (distance <= 0.5) {
        distance = 500000;
    }

#if _DEBUG_FILTRATION
    std::cout << "Dynamic Kalman: (a_data.cos_v - m_filtrated_data.angle) = " << a_data.cos_v - m_filtrated_data.angle << "  distance = " << distance << std::endl;
#endif

    return distance;
}

Eigen::MatrixXd dynamic_kalman_filter::get_Q(const double &distance) {
    Eigen::MatrixXd Q_tmp(2, 2);
    Q_tmp << get_Q_a(distance), get_Q_b(distance), get_Q_b(distance), get_Q_c(distance);

    return Q_tmp;
}

double dynamic_kalman_filter::get_Q_a(const double &distance) {
    return 0.9999651 / std::pow(distance, 1.8599);
}

double dynamic_kalman_filter::get_Q_b(const double &distance) {
    return 0.9946035 / std::pow(distance, 1.6995);
}

double dynamic_kalman_filter::get_Q_c(const double &distance) {
    return 0.99912324 / std::pow(distance, 1.63099);
}


/*----------------------------- KALMAN BANK -----------------------------*/

bank_of_kalman_filters::bank_of_kalman_filters(const std::vector<static_kalman_filter> &a_filters) : m_filters(
        a_filters) {}

void bank_of_kalman_filters::add_filter(const static_kalman_filter &a_filter) {
    m_filters.push_back(a_filter);
}

bool bank_of_kalman_filters::filtrate(const measurement_data &a_data) {
    auto end_it = m_filters.end();
    for (auto it = m_filters.begin(); it != end_it; ++it) {
        it->filtrate(a_data);
    }
}
