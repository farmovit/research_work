#pragma once

#include <memory>

#include "initializer.h"
#include "pugixml.hpp"

class abstract_initializer_factory {
public:
    abstract_initializer_factory() {}

    virtual void parse(const pugi::xml_node &node) = 0;

    virtual std::shared_ptr<abstract_initializer> create() = 0;
};


class initializer_by_points_factory : public abstract_initializer_factory {
public:
    initializer_by_points_factory() {}

    virtual void parse(const pugi::xml_node &node);

    virtual std::shared_ptr<abstract_initializer> create();

private:
    std::size_t m_points;
};
