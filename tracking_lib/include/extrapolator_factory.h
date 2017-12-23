#pragma once

#include <memory>
#include "pugi_tools.h"
#include "extrapolator.h"

class abstract_extrapolator_factory
{
public:
	abstract_extrapolator_factory(){}
	
	virtual void parse(const pugi::xml_node &node) = 0;
	
	virtual std::shared_ptr<abstract_extrapolator> create() = 0;
};

class last_point_extrapolator_factory: public abstract_extrapolator_factory
{
public:
	last_point_extrapolator_factory(){}
	
	virtual void parse(const pugi::xml_node &node);
	
	virtual std::shared_ptr<abstract_extrapolator> create();
};

class linear_extrapolator_factory: public abstract_extrapolator_factory
{
public:
	linear_extrapolator_factory(){}
	
	virtual void parse(const pugi::xml_node &node);
	
	virtual std::shared_ptr<abstract_extrapolator> create();
};
