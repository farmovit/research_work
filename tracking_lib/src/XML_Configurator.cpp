#include "XML_Configurator.h"

void XML_Configurator::parse(const std::string &xml_name) {
	pugi::xml_document xml_doc;
	pugi::xml_parse_result parse_result = xml_doc.load_file(xml_name.c_str());
	if (!parse_result) {
		std::stringstream err;
		err << "Error: parsing " << xml_name << " is failed: " << parse_result.description();
		throw std::runtime_error(err.str());
	}
	
	pugi::xml_node root_node = xml_doc.child("root");
	if (!root_node) {
		std::stringstream err;
		err << "Error: parsing " << xml_name << " is failed: root node is not found";
		throw std::runtime_error(err.str());
	}  
	
	pugi::xml_child tracking_object_node(root_node);
	
	m_tracking_object_factory_creator = std::make_shared<tracking_object_factory_creator>();
  	m_tracking_object_factory_creator->create(tracking_object_node("tracking_object"));
	
	pugi::xml_child identifier_node(root_node);

	//TODO: rewrite this
	create_identifier(identifier_node("identifier"));
}

void XML_Configurator::create_identifier(const pugi::xml_node& node) {
	pugi::xml_child child(node);
	
	m_identifier = std::make_shared<identifier>();
	double residual = atof(child("res").child_value());
	m_identifier->set_checker(get_identifier_checkers(residual));
}

std::shared_ptr<identification_checker_base> XML_Configurator::get_identifier_checkers(const double &res) {
	std::shared_ptr<identification_checker_base> checkers =
			std::make_shared<identification_checker_residual>(res);
	checkers->add_checker(std::make_shared<identification_checker_type>());
  	checkers->add_checker(std::make_shared<identification_checker_alive>());
  
  	return checkers;
}

std::shared_ptr<tracking_object_factory_creator> XML_Configurator::get_tracking_object_factory_creator() const {
	return m_tracking_object_factory_creator;
}
	
std::shared_ptr<identifier> XML_Configurator::get_identifier() const {
	return m_identifier;
}
