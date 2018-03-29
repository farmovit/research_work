#pragma once

#include <memory>

#include "filter.h"
#include "pugixml.hpp"
#include "pugi_tools.h"

#include <Eigen/Dense>

class abstract_filter_factory
{
public:
	abstract_filter_factory(){}
	virtual void parse(const pugi::xml_node &node) = 0;
	virtual std::shared_ptr<abstract_filter> create() = 0;
};


class alpha_beta_filter_factory: public abstract_filter_factory
{
public:
	alpha_beta_filter_factory(){}
	virtual void parse(const pugi::xml_node &node);
	virtual std::shared_ptr<abstract_filter> create();

private:
	double m_alpha;
	double m_beta;
};


class alpha_beta_gamma_filter_factory: public abstract_filter_factory
{
public:
	alpha_beta_gamma_filter_factory(){}
	virtual void parse(const pugi::xml_node &node);
	virtual std::shared_ptr<abstract_filter> create();

private:
	double m_alpha;
	double m_beta;
	double m_gamma;
};


class static_kalman_filter_factory: public abstract_filter_factory
{
public:
	static_kalman_filter_factory(){}
	virtual void parse(const pugi::xml_node &node);
	virtual std::shared_ptr<abstract_filter> create();

	Eigen::MatrixXd get_H() const { return m_H; }
	Eigen::MatrixXd get_R() const { return m_R; }
	Eigen::MatrixXd get_F() const { return m_F; }
	Eigen::MatrixXd get_Q() const { return m_Q; }
	Eigen::MatrixXd get_P() const { return m_P; }
	Eigen::VectorXd get_Xk_() const { return m_Xk_; }
	
private:

	Eigen::MatrixXd m_H;
	Eigen::MatrixXd m_R;
	Eigen::MatrixXd m_F;
	Eigen::MatrixXd m_Q;
	Eigen::MatrixXd m_P;
	Eigen::VectorXd m_Xk_;
};


class dynamic_kalman_filter_factory: public abstract_filter_factory
{
public:
  dynamic_kalman_filter_factory(){}
  
  virtual void parse(const pugi::xml_node &node);
  
  virtual std::shared_ptr<abstract_filter> create();

private:
	double m_velocity;
	static_kalman_filter_factory m_static_kalman_factory;
};
