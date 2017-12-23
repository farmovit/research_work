#include "initializer_factory.h"

#include <memory>
#include <iostream>
#include "initializer.h"
#include "pugixml.hpp"
#include "pugi_tools.h"

#define _Debug_initializer_factories 0

/**************** initializer by points factory factory *****************/

void initializer_by_points_factory::parse(const pugi::xml_node &node)
{
#if _Debug_initializer_factories
  std::cout << "Parsing params in xml for initializer by points" << std::endl;
#endif
	
	pugi::xml_child child(node);
	
	m_points = atoi(child("points").child_value());
}

std::shared_ptr<abstract_initializer> initializer_by_points_factory::create()
{
#if _Debug_initializer_factories
  std::cout << "Creating initializer by " << m_points << " points" << std::endl;
#endif

  return std::make_shared<initializer_by_points>(m_points);
}