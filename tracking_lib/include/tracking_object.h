#pragma once

#include "initializer.h"
#include "structions.h"
#include "filter.h"
#include "extrapolator.h"

#include <memory>
#include <list>
#include <iterator>


struct tracking_object_params {
    tracking_object_params() : alive(true) {};

    double time_last;
    double delta_time; //time
    double angle; //pel
    double speed;
    double acceleration;
    int type; //type
    bool alive; //has data
    bool is_init;
    std::size_t out_idx;
    std::size_t internal_idx;

    // FIXME: these angles and times are used to debug only
    // and must be deleted in production
    std::vector<double> v_angles;
    std::vector<double> v_times;
};

class tracking_object {
public:
    tracking_object() :
            m_initializer(nullptr),
            m_filter(nullptr),
            m_extrapolator(nullptr) {}

    ~tracking_object() {}

    tracking_object_params get_params() const;

    bool is_alive() const;

    bool is_initialized() const;

    int get_type() const;

    std::size_t get_number() const;

    std::size_t get_out_number() const;

    double get_time() const;

    double get_angle() const;

    void kill();

    void extrapolate(const measurement_data &);

    void update(const measurement_data &);

    void set_initializer(std::shared_ptr<abstract_initializer>);

    void set_filter(std::shared_ptr<abstract_filter>);

    void set_extrapolator(std::shared_ptr<abstract_extrapolator>);

    void set_type(const int &);

    void set_number(const std::size_t &);

    void set_out_number(const std::size_t &);

private:
    tracking_object_params m_params;

    std::shared_ptr<abstract_initializer> m_initializer;
    std::shared_ptr<abstract_filter> m_filter;
    std::shared_ptr<abstract_extrapolator> m_extrapolator;

    void set_data(const filter_params &);

    void set_time(const double &);
};

typedef std::shared_ptr<tracking_object> tracking_object_ptr;
typedef std::list<tracking_object_ptr> tracking_object_storage;
typedef tracking_object_storage::iterator tracking_object_iterator;
