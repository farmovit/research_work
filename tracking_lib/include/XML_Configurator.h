#pragma once

#include <memory>
#include <string>

#include "factory_creator.h"
#include "tracking_object_factory.h"
#include "identifier.h"
#include "identifier_checker.h"
#include "wrapped_factory.h"

#include "pugixml.hpp"


class XML_Configurator
{
public:
  XML_Configurator()
		: m_tracking_object_factory_creator(nullptr) {}

	void parse(const std::string & );
	
	std::shared_ptr<tracking_object_factory_creator> get_tracking_object_factory_creator() const;
	
	std::shared_ptr<identifier> get_identifier() const;
	
private:
	void create_identifier(const pugi::xml_node& );
	std::shared_ptr<identification_checker_base> get_identifier_checkers(const double &res);

	std::shared_ptr<tracking_object_factory_creator> m_tracking_object_factory_creator;
	std::shared_ptr<identifier> m_identifier;
};
