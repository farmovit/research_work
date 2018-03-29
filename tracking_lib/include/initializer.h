#pragma once

#include "structions.h"

const double c_init_time = 0.03;

class abstract_initializer {

protected:
	bool m_is_initialize;

public:
	abstract_initializer();
	
	bool check_state() const;
	void set_state(bool );
  
	virtual filter_params get_filter_param() = 0;
	virtual bool initialize(const measurement_data&) = 0;
	
	virtual ~abstract_initializer() {}
};

class initializer_by_points : public abstract_initializer {
	
public:
	initializer_by_points(std::size_t );
	filter_params get_filter_param();
	bool initialize(const measurement_data&);
  
	virtual ~initializer_by_points() {}

private:
	bool add_data(const measurement_data &);
	void estimate_params();
	
	bool calculate_speed(double* );
	bool calculate_acceleration(double* );
	bool calculate_dtime(double* );
	
	peleng_data_storage m_init_vec;
	filter_params m_initialized_data;
	std::size_t m_init_counter;
	bool m_need_acceleration;
};
