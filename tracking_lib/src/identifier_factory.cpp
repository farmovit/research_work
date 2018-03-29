#include "identifier_factory.h"

#include <memory>
#include <iostream>
#include "pugixml.hpp"
#include "identifier.h"

void all_target_identifier_factory::parse(const pugi::xml_node &node) {
    std::cout << "Parsing params in xml for identifier" << std::endl;
}

std::shared_ptr<abstract_identifier_factory> all_target_identifier_factory::create()
{
    std::cout << "Creating identifier" << std::endl;
    return std::make_shared<all_target_identifier_factory>();
}
