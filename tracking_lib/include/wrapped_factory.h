#pragma once

#include <memory>
#include <iostream>

#include "pugixml.hpp"
#include "filter_factory.h"
#include "extrapolator_factory.h"
#include "initializer_factory.h"
#include "factory_creator.h"


template<class T>
class abstract_wrapped_factory {
private:
    std::shared_ptr<abstract_factory_creator<T> > m_creator;

protected:
    abstract_wrapped_factory() : m_creator(nullptr) {}

    void add(std::shared_ptr<abstract_factory_creator<T> > ptr) {
        if (m_creator == nullptr) m_creator = ptr;
        else m_creator->add(ptr);
    }

public:
    ~abstract_wrapped_factory() {}

    std::shared_ptr<T> create(const pugi::xml_node &node) {
        auto temp = m_creator->create(node);
        if (temp == nullptr) {
            throw std::runtime_error("Can't create object");
        }
        return temp;
    }
};

class filter_wrapped_factory : public abstract_wrapped_factory<abstract_filter_factory> {
public:
    filter_wrapped_factory() {
        add(std::make_shared<factory_creator<abstract_filter_factory, alpha_beta_filter_factory> >("alpha_beta"));
        add(std::make_shared<factory_creator<abstract_filter_factory, alpha_beta_gamma_filter_factory> >(
                "alpha_beta_gamma"));
        add(std::make_shared<factory_creator<abstract_filter_factory, static_kalman_filter_factory> >("static_kalman"));
        add(std::make_shared<factory_creator<abstract_filter_factory, dynamic_kalman_filter_factory> >(
                "dynamic_kalman"));
    }
};

class extrapolator_wrapped_factory : public abstract_wrapped_factory<abstract_extrapolator_factory> {
public:
    extrapolator_wrapped_factory() {
        add(std::make_shared<factory_creator<abstract_extrapolator_factory, last_point_extrapolator_factory> >(
                "last_point_extrapolator"));
        add(std::make_shared<factory_creator<abstract_extrapolator_factory, linear_extrapolator_factory> >(
                "linear_extrapolator"));
    }
};

class initializer_wrapped_factory : public abstract_wrapped_factory<abstract_initializer_factory> {
public:
    initializer_wrapped_factory() {
        add(std::make_shared<factory_creator<abstract_initializer_factory, initializer_by_points_factory> >(
                "by_points"));
    }
};
