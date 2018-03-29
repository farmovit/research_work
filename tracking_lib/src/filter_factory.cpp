#include "filter_factory.h"

#include <iosfwd>

#include "tools.h"


#define _DEBUG_FILTER_FACTORY 0


/**************** alpha-beta factory *****************/

void alpha_beta_filter_factory::parse(const pugi::xml_node &node) {
#if _DEBUG_FILTER_FACTORY
  std::cout << "Parsing params in xml for alpha-beta filter" << std::endl;
#endif
	
	pugi::xml_child child(node);
	
	m_alpha = atof(child("alpha").child_value());
	m_beta = atof(child("beta").child_value());		
}

std::shared_ptr<abstract_filter> alpha_beta_filter_factory::create() {
#if _DEBUG_FILTER_FACTORY
  std::cout << "Creating alpha-beta filter with parameters: alpha=" << m_alpha << " beta=" << m_beta << std::endl;
#endif

	return std::make_shared<alpha_beta_filter>(m_alpha, m_beta);
}

/**************** alpha-beta-gamma factory *****************/

void alpha_beta_gamma_filter_factory::parse(const pugi::xml_node &node) {
#if _DEBUG_FILTER_FACTORY
  std::cout << "Parsing params in xml for alpha-beta-gamma filter" << std::endl;
#endif
	pugi::xml_child child(node);
	
	m_alpha = atof(child("alpha").child_value());
	m_beta = atof(child("beta").child_value());
	m_gamma = atof(child("gamma").child_value());
}

std::shared_ptr<abstract_filter> alpha_beta_gamma_filter_factory::create() {
#if _DEBUG_FILTER_FACTORY
  std::cout << "Creating alpha-beta-gamma filter with parameters: alpha=" << m_alpha << " beta=" << m_beta << " gamma=" << m_gamma << std::endl;
#endif

  return std::make_shared<alpha_beta_gamma_filter>(m_alpha, m_beta, m_gamma);
}

/**************** static kalman factory *****************/

void static_kalman_filter_factory::parse(const pugi::xml_node &node) {
#if _DEBUG_FILTER_FACTORY
  std::cout << "Parsing params in xml for static kalman filter" << std::endl;
#endif

	pugi::xml_child child(node);

	m_H = tracking_tools::construct_matrix(child("H"));
	m_R = tracking_tools::construct_matrix(child("R"));
	m_F = tracking_tools::construct_matrix(child("F"));
	m_Q = tracking_tools::construct_matrix(child("Q"));
	m_P = tracking_tools::construct_matrix(child("P"));
	m_Xk_ = tracking_tools::construct_vector(child("Xk_"));
}

std::shared_ptr<abstract_filter> static_kalman_filter_factory::create() {
#if _DEBUG_FILTER_FACTORY
  std::cout << "Creating static kalman filter" << std::endl;
#endif

  return std::make_shared<static_kalman_filter>(m_H, m_R, m_F, m_Q, m_P, m_Xk_);
}

/**************** dynamic kalman factory *****************/

void dynamic_kalman_filter_factory::parse(const pugi::xml_node &node) {
#if _DEBUG_FILTER_FACTORY
	std::cout << "Parsing params in xml for dynamic kalman filter" << std::endl;
#endif

	m_static_kalman_factory.parse(node);
 
	pugi::xml_child child(node);
	
	m_velocity = atof(child("velocity").child_value());
}

std::shared_ptr<abstract_filter> dynamic_kalman_filter_factory::create() {
#if _DEBUG_FILTER_FACTORY
  std::cout << "Creating dynamic kalman filter" << std::endl;
#endif
	
	auto static_kalman = static_kalman_filter(
		m_static_kalman_factory.get_H(),
		m_static_kalman_factory.get_R(),
		m_static_kalman_factory.get_F(),
		m_static_kalman_factory.get_Q(),
		m_static_kalman_factory.get_P(),
		m_static_kalman_factory.get_Xk_()
	);

  return std::make_shared<dynamic_kalman_filter>(static_kalman, m_velocity);
}
