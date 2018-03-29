#pragma once

#include <memory>
#include "pugixml.hpp"
#include "identifier.h"

class abstract_identifier_factory {
public:
    abstract_identifier_factory() {}

    virtual void parse(const pugi::xml_node &node) = 0;

    virtual std::shared_ptr<abstract_identifier_factory> create() = 0;
};


class all_target_identifier_factory : public abstract_identifier_factory {
public:
    all_target_identifier_factory() {}

    virtual void parse(const pugi::xml_node &node);

    virtual std::shared_ptr<abstract_identifier_factory> create();
};
