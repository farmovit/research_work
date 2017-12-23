#include "extrapolator_factory.h"

#include <memory>
#include <iostream>
#include "pugi_tools.h"
#include "extrapolator.h"


void last_point_extrapolator_factory::parse(const pugi::xml_node &node) 
{
    std::cout << "No needed parsing parameters for last point extrapolator" << std::endl;
}

std::shared_ptr<abstract_extrapolator> last_point_extrapolator_factory::create()
{
    std::cout << "Creating last point extrapolator" << std::endl;
		return std::make_shared<last_point_extrapolator>();
}


void linear_extrapolator_factory::parse(const pugi::xml_node &node)
{
    std::cout << "No needed parsing parameters for linear extrapolator" << std::endl;
}

std::shared_ptr<abstract_extrapolator> linear_extrapolator_factory::create()
{
    std::cout << "Creating linear extrapolator" << std::endl;
		return std::make_shared<linear_extrapolator>();
}
