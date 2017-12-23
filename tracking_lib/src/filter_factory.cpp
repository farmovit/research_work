#include "filter_factory.h"

#include <memory>
#include <iostream>
#include <sstream>
#include <vector>

#include "filter.h"
#include "pugixml.hpp"
#include "pugi_tools.h"
#include "tools.h"

#define _DEBUG_FILTER_FACTORY 0


/**************** alpha-beta factory *****************/

void alpha_beta_filter_factory::parse(const pugi::xml_node &node)
{
#if _DEBUG_FILTER_FACTORY
  std::cout << "Parsing params in xml for alpha-beta filter" << std::endl;
#endif
	
	pugi::xml_child child(node);
	
	m_alpha = atof(child("alpha").child_value());
	m_beta = atof(child("beta").child_value());		
}

std::shared_ptr<abstract_filter> alpha_beta_filter_factory::create()
{
#if _DEBUG_FILTER_FACTORY
  std::cout << "Creating alpha-beta filter with parameters: alpha=" << m_alpha << " beta=" << m_beta << std::endl;
#endif

	return std::make_shared<alpha_beta_filter>(m_alpha, m_beta);
}

/**************** alpha-beta-gamma factory *****************/

void alpha_beta_gamma_filter_factory::parse(const pugi::xml_node &node)
{
#if _DEBUG_FILTER_FACTORY
  std::cout << "Parsing params in xml for alpha-beta-gamma filter" << std::endl;
#endif
	pugi::xml_child child(node);
	
	m_alpha = atof(child("alpha").child_value());
	m_beta = atof(child("beta").child_value());
	m_gamma = atof(child("gamma").child_value());
}

std::shared_ptr<abstract_filter> alpha_beta_gamma_filter_factory::create()
{
#if _DEBUG_FILTER_FACTORY
  std::cout << "Creating alpha-beta-gamma filter with parameters: alpha=" << m_alpha << " beta=" << m_beta << " gamma=" << m_gamma << std::endl;
#endif

  return std::make_shared<alpha_beta_gamma_filter>(m_alpha, m_beta, m_gamma);
}

/**************** static kalman factory *****************/

void static_kalman_filter_factory::parse(const pugi::xml_node &node)
{
#if _DEBUG_FILTER_FACTORY
  std::cout << "Parsing params in xml for static kalman filter" << std::endl;
#endif

	pugi::xml_child child(node);
	
	m_H = get_matrix(child("H"));
	m_R = get_matrix(child("R"));
	m_F = get_matrix(child("F"));
	m_Q = get_matrix(child("Q"));
	m_P = get_matrix(child("P"));
	m_Xk_ = get_vector(child("Xk_"));
}

std::shared_ptr<abstract_filter> static_kalman_filter_factory::create()
{
#if _DEBUG_FILTER_FACTORY
  std::cout << "Creating static kalman filter" << std::endl;
#endif

  return std::make_shared<static_kalman_filter>(m_H, m_R, m_F, m_Q, m_P, m_Xk_);
}

Eigen::MatrixXd static_kalman_filter_factory::get_matrix(const pugi::xml_node &node)
{
	pugi::xml_child child(node);
	
	std::size_t M = atoi(child("M").child_value());
	std::size_t N = atoi(child("N").child_value());
	std::string values = child("values").child_value();
	
	std::vector<double> v_values;
	
  std::stringstream ssin(values);
	double tmp;
  while (ssin >> tmp) {
		v_values.push_back(tmp);
  }
	
	Eigen::MatrixXd result_matrix;
	if(!v_values.empty()) {
		result_matrix = makeMatrix(M, N, v_values);
	}
	else {
		result_matrix = Eigen::MatrixXd::Zero(M, N);
	}
	
#if _DEBUG_FILTER_FACTORY
  std::cout << node.name() << " = " << result_matrix << std::endl;
#endif
	
	return result_matrix;
}

Eigen::VectorXd static_kalman_filter_factory::get_vector(const pugi::xml_node &node)
{
	pugi::xml_child child(node);
	
	std::size_t M = atoi(child("M").child_value());
	std::size_t N = atoi(child("N").child_value());
	std::string values = child("values").child_value();
	
	std::vector<double> v_values;
	
  std::stringstream ssin(values);
	double tmp;
  while (ssin >> tmp) {
		v_values.push_back(tmp);
  }
	
	Eigen::VectorXd result_vector;
	if(!v_values.empty()) {
		result_vector = makeVector(v_values);
	}
	else {
		result_vector = Eigen::VectorXd::Zero(M*N);
	}
	
#if _DEBUG_FILTER_FACTORY
  std::cout << node.name() << " = " << result_vector << std::endl;
#endif
	
	return result_vector;
}

/**************** dynamic kalman factory *****************/

void dynamic_kalman_filter_factory::parse(const pugi::xml_node &node)
{
#if _DEBUG_FILTER_FACTORY
	std::cout << "Parsing params in xml for dynamic kalman filter" << std::endl;
#endif

	m_static_kalman_factory.parse(node);
 
	pugi::xml_child child(node);
	
	m_velocity = atof(child("velocity").child_value());
}

std::shared_ptr<abstract_filter> dynamic_kalman_filter_factory::create()
{
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
