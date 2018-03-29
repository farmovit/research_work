#include "tracking_object_factory.h"

#include "filter_factory.h"
#include "initializer_factory.h"
#include "extrapolator_factory.h"
#include "pugixml.hpp"
#include "pugi_tools.h"

#include <memory>

std::shared_ptr<tracking_object> tracking_object_factory::get_tracking_object() {
	auto object = std::make_shared<tracking_object>();
	object->set_filter(m_filter_factory->create());
	object->set_extrapolator(m_extrapolator_factory->create());
	object->set_initializer(m_initializer_factory->create());
	return object;
}

void tracking_object_factory::set_filter_factory(std::shared_ptr<abstract_filter_factory> a_filter_factory) {
	m_filter_factory = a_filter_factory;
}

void tracking_object_factory::set_extrapolator_factory(std::shared_ptr<abstract_extrapolator_factory> a_extrapolator_factory) {
	m_extrapolator_factory = a_extrapolator_factory;
}

void tracking_object_factory::set_initializer_factory(std::shared_ptr<abstract_initializer_factory> a_initializer_factory) {
	m_initializer_factory = a_initializer_factory;
}

void tracking_object_factory_creator::create(const pugi::xml_node &xml_node) {
	m_tracking_object_factory = std::make_shared<tracking_object_factory>();
	
	pugi::xml_child child(xml_node);
 	m_tracking_object_factory->set_filter_factory( m_filter_factory_creator.create(child("Filter")));
	m_tracking_object_factory->set_extrapolator_factory( m_extrapolator_factory_creator.create(child("Extrapolator")));
	m_tracking_object_factory->set_initializer_factory( m_initializer_factory_creator.create(child("Initializer")));
}

std::shared_ptr<tracking_object_factory> tracking_object_factory_creator::get_factory() const {
	return m_tracking_object_factory;
}
