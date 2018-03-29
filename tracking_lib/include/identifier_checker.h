#pragma once

#include <memory>

#include "tracking_object.h"

class identification_checker_base {
public:
    bool check(const tracking_object &object, const measurement_data &data) {
        if (check_self(object, data)) return check_next(object, data);
        else return false;
    }

    void add_checker(std::shared_ptr<identification_checker_base> a_checker) {
        if (m_next_checker == nullptr) m_next_checker = a_checker;
        else m_next_checker->add_checker(a_checker);
    }

    identification_checker_base() : m_next_checker(nullptr) {}

    virtual ~identification_checker_base();

protected:
    virtual bool check_self(const tracking_object &object, const measurement_data &data) = 0;

    bool check_next(const tracking_object &object, const measurement_data &data) {
        if (m_next_checker == nullptr) return true;
        else return m_next_checker->check(object, data);
    }

private:
    std::shared_ptr<identification_checker_base> m_next_checker;
};

class identification_checker_residual : public identification_checker_base {
public:
    identification_checker_residual(double);

    ~identification_checker_residual();

private:
    bool check_self(const tracking_object &object, const measurement_data &data);

    double m_limit;
};

class identification_checker_type : public identification_checker_base {
public:
    identification_checker_type();

    ~identification_checker_type();

private:
    bool check_self(const tracking_object &object, const measurement_data &data);
};

class identification_checker_alive : public identification_checker_base {
public:
    identification_checker_alive();

    ~identification_checker_alive();

private:
    bool check_self(const tracking_object &object, const measurement_data &data);
};
