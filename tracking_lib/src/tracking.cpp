#include "tracking.h"
#include <fstream>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

#define _DEBUG_TRACKING 0

struct is_old {
    is_old(const double &a_time, const int &a_res)
            : m_time(a_time), m_residual(a_res) {}

    bool operator()(tracking_object_ptr track) {
        if (!track->is_initialized())
            return false;

        return (fabs(m_time - track->get_time()) > m_residual);
    }

    double m_time;
    int m_residual;
};

struct Minimal_res {
    Minimal_res(const double &a_pel) :
            m_pel(a_pel), m_res(3 * C_SIGMA_PEL) {};

    void operator()(identified_object_data &a_data) {
        double temp_res = fabs(a_data.m_tracking_object->get_params().angle - m_pel);

        if (temp_res < m_res) {
            m_res = temp_res;
            result_data = a_data;
        }
    }

    identified_object_data result_data;
    double m_pel;
    double m_res;
};

tracking::tracking(std::shared_ptr<identifier> a_identifier,
                   std::shared_ptr<tracking_object_factory> a_tracking_object_factory)
        :
        m_identifier(a_identifier),
        m_tracking_object_factory(a_tracking_object_factory),
        m_out_number(C_MIN_PEL_OUT_NUM) {}

tracking::~tracking() {}

void tracking::reset() {
    if (m_tracked_object_list.empty()) {
        return;
    }

    m_tracked_object_list.erase(m_tracked_object_list.begin(), m_tracked_object_list.end());
}

std::size_t tracking::get_new_number() {
    if (m_out_number >= C_MAX_PEL_OUT_NUM) {
        m_out_number = C_MIN_PEL_OUT_NUM;
    }

    return m_out_number++;
}

void tracking::delete_old(const double &a_time) {
    if (m_tracked_object_list.empty()) {
        return;
    }

    m_tracked_object_list.remove_if(is_old(a_time, C_DELTA_T_OLD));
}


tracking_object_storage tracking::get_trecked_object_list() const {
    return m_tracked_object_list;
}


void tracking::go(const measurement_data &a_data) {
    if (m_tracked_object_list.empty()) {
        std::cout << "new" << std::endl;
        add_new_object(a_data);
        return;
    }

    identified_object_storage_ptr identified_objects;
    identified_objects = m_identifier->identify(m_tracked_object_list, a_data);

    if (identified_objects->empty()) {
        add_new_object(a_data);
        std::cout << "object was not identified" << std::endl;
        return;
    }

    Minimal_res temp_obj = for_each(identified_objects->begin(),
                                    identified_objects->end(),
                                    Minimal_res(a_data.cos_v));

    auto current_object = temp_obj.result_data.m_tracking_object;
    current_object->update(a_data);
    set_current_object(*current_object);
}

void tracking::add_new_object(const measurement_data &from) {
#if _DEBUG_TRACKING
    std::cout << "INFO: Creating new object" << std::endl;
#endif

    std::shared_ptr<tracking_object> tracking_object_ = m_tracking_object_factory->get_tracking_object();

    tracking_object_->set_type(from.type);
    tracking_object_->set_number(m_tracked_object_list.size());
    tracking_object_->set_out_number(get_new_number());
    tracking_object_->update(from);

    m_tracked_object_list.push_back(tracking_object_);
    set_current_object(*tracking_object_);
}

void tracking::set_current_object(const tracking_object &object) {
    m_current_tracked_object = object;
}

tracking_object tracking::get_tracked_object() const {
    return m_current_tracked_object;
}
