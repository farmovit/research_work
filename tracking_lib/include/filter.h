#pragma once

#include "structions.h"

#include <cmath>
#include <functional>

#include <Eigen/Dense>


class abstract_filter 
{
public:
	abstract_filter(bool);
	
	bool check_state() const;
	virtual filter_params get_filtrated_data();
	
	virtual void initialize(const filter_params& params) = 0;
	virtual bool filtrate(const measurement_data& measurement) = 0;
	
	virtual ~abstract_filter() {};

protected:
	bool m_is_initialize;
	filter_params m_filtrated_data;
	bool check_time(const measurement_data& input_data);

};


class alpha_beta_filter : public abstract_filter 
{	
public:
	alpha_beta_filter(
		const double& a_alpha, 
		const double& a_beta
	);

	void initialize(const filter_params& );
	bool filtrate(const measurement_data& );
	
	virtual ~alpha_beta_filter() {};

private:
	double m_alpha;
	double m_beta;
};


class alpha_beta_gamma_filter : public abstract_filter 
{
public:
	alpha_beta_gamma_filter(
		const double& a_alpha, 
		const double& a_beta, 
		const double& a_gamma
	);
	
	void initialize(const filter_params& );
	bool filtrate(const measurement_data& );
	
	virtual ~alpha_beta_gamma_filter() {};

private:
	double m_alpha;
	double m_beta;
	double m_gamma;
};


class static_kalman_filter : public abstract_filter {
	
public:
	static_kalman_filter(
		const Eigen::MatrixXd& a_H,
		const Eigen::MatrixXd& a_R,
		const Eigen::MatrixXd& a_F,
		const Eigen::MatrixXd& a_Q,
		const Eigen::MatrixXd& a_P,
		const Eigen::VectorXd& a_Xk_
	);
	
	void initialize(const filter_params& );
	bool filtrate(const measurement_data& );
	
	virtual ~static_kalman_filter() {};

	void set_H(const Eigen::MatrixXd& );
	void set_R(const Eigen::MatrixXd& );
	void set_Xk_(const Eigen::VectorXd& );
	void set_Xk_(const double& cos_a, const double& dcos_a);
	void set_F(const Eigen::MatrixXd& );
	void set_F(const double& );
	void set_Q(const Eigen::MatrixXd& );
	void set_P(const Eigen::MatrixXd& );

	void PrintF();
	void PrintH();
	void PrintQ();
	void PrintR();
	void PrintP();
	void PrintXk_();
	
protected:
	
	Eigen::MatrixXd m_F;
	Eigen::MatrixXd m_H;
	Eigen::MatrixXd m_Q;
	Eigen::MatrixXd m_R;
	Eigen::MatrixXd m_P;
	Eigen::MatrixXd m_Xk_;
};


class dynamic_kalman_filter: public abstract_filter
{

public:
	dynamic_kalman_filter(
		const Eigen::MatrixXd& a_H,
		const Eigen::MatrixXd& a_R,
		const Eigen::MatrixXd& a_F,
		const Eigen::MatrixXd& a_Q,
		const Eigen::MatrixXd& a_P,
		const Eigen::VectorXd& a_Xk_,
		const double& a_velocity
	);
	
	dynamic_kalman_filter(
		const static_kalman_filter& a_kalman, 
		const double& a_velocity
	);
	
	void initialize(const filter_params& );
	bool filtrate(const measurement_data& );
	
private:
	double get_Q_a(const double& );
	double get_Q_b(const double& );
	double get_Q_c(const double& );
	Eigen::MatrixXd get_Q(const double& );
	double get_distance(const measurement_data& );

	
	double m_velocity;
	static_kalman_filter m_static_kalman;
};


class bank_of_kalman_filters: public abstract_filter
{
public:
	bank_of_kalman_filters(
		const std::vector<static_kalman_filter>& a_filters
	);
	
	void add_filter(const static_kalman_filter& );
	bool filtrate(const measurement_data& );
	
private:
	std::vector<static_kalman_filter> m_filters;
	
};
