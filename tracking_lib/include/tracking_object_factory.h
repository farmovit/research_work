#pragma once

#include "wrapped_factory.h"
#include "filter_factory.h"
#include "initializer_factory.h"
#include "extrapolator_factory.h"
#include "tracking_object.h"
#include "pugixml.hpp"

#include <memory>

class tracking_object_factory
{
public:
	tracking_object_factory() :
		m_filter_factory(nullptr),
		m_extrapolator_factory(nullptr),
		m_initializer_factory(nullptr)
	{}

	std::shared_ptr<tracking_object> get_tracking_object();
	void set_filter_factory(std::shared_ptr<abstract_filter_factory> );
	void set_extrapolator_factory(std::shared_ptr<abstract_extrapolator_factory> );
	void set_initializer_factory(std::shared_ptr<abstract_initializer_factory> );
	
private:
	std::shared_ptr<abstract_filter_factory> m_filter_factory;
	std::shared_ptr<abstract_extrapolator_factory> m_extrapolator_factory;
	std::shared_ptr<abstract_initializer_factory> m_initializer_factory;
};

class tracking_object_factory_creator
{
public:
	tracking_object_factory_creator() :
		m_tracking_object_factory(nullptr)
  	{}

	void create(const pugi::xml_node & );
	std::shared_ptr<tracking_object_factory> get_factory() const;
	
private:
	std::shared_ptr<tracking_object_factory> m_tracking_object_factory;
  
	filter_wrapped_factory m_filter_factory_creator;
  	extrapolator_wrapped_factory m_extrapolator_factory_creator;
  	initializer_wrapped_factory m_initializer_factory_creator;
};