#include "initializer.h"
#include "tools.h"

#define _DEBUG_INITIALIZER 0

using namespace std;

abstract_initializer::abstract_initializer() : m_is_initialize(false)
{}

bool abstract_initializer::check_state() const {
	return m_is_initialize;
}

void abstract_initializer::set_state(bool flag) {
	m_is_initialize = flag;
}

initializer_by_points::initializer_by_points(std::size_t a_init_counter) : m_init_counter(a_init_counter) {
	if(a_init_counter > 2) {
		m_need_acceleration = true;
	}
}

filter_params initializer_by_points::get_filter_param() {
	return m_initialized_data;
}

bool initializer_by_points::initialize(const measurement_data& from) {
  	if(check_state())	return true;

	m_initialized_data.time_last = from.time;
	m_initialized_data.angle     = from.cos_v;
	m_initialized_data.speed     = 0.0;
	m_initialized_data.acceleration = 0.0;
	m_initialized_data.delta_time = 0.0;
	
  	if(!add_data(from)) return false;
	if (m_init_vec.size() < m_init_counter) return false;
  
	estimate_params();
	return check_state();
}

bool initializer_by_points::add_data(const measurement_data& from) {
	if(m_init_vec.empty()) {
    	m_init_vec.push_back(from);
    	return true;
  	}

	auto last = m_init_vec.end();
	--last;
	
  	if (fabs(from.time - last->time) > c_init_time) {
    	m_init_vec.push_back(from);
    	return true;
  	}

	return false;
}

void initializer_by_points::estimate_params() {
	bool state = false;
	
	double init_speed;
	double init_acceleration;
	double init_dtime;
	
	if(!calculate_dtime(&init_dtime)) {
		set_state(state);
		return;
	}
	
	if(!calculate_speed(&init_speed)) {
		set_state(state);
		return;
	}
	
	if(m_need_acceleration) {
		if(!calculate_acceleration(&init_acceleration)) {
			set_state(state);
			return;
		}
	}
	
	auto last_data = m_init_vec.end() - 1;
	
	m_initialized_data.time_last  = last_data->time;
	m_initialized_data.angle      = last_data->cos_v;
	m_initialized_data.delta_time = init_dtime;
	m_initialized_data.speed      = init_speed;
	m_initialized_data.acceleration = init_acceleration;
	
#if _DEBUG_INITIALIZER  
	printf("Init_time: %f\n", m_initialized_data.time_last);
	printf("Init_cos_v: %f\n", m_initialized_data.angle);
	printf("Init_speed: %f\n", m_initialized_data.speed);
	printf("Init_acceler: %f\n", m_initialized_data.acceleration);
#endif
	
	state = true;
	set_state(state);	  
}

bool initializer_by_points::calculate_speed(double* a_speed) {
	auto last  = m_init_vec.end() - 1;
	auto first = m_init_vec.end() - 2;

	double speed = (last->cos_v - first->cos_v) / (last->time - first->time);
	
	if ( (fabs(speed) < COMMON_EPS) || (fabs(speed) > 1e4) ) {
		return false;
	}
	
	*a_speed = speed;
	return true;
}

bool initializer_by_points::calculate_dtime(double* a_dtime) {
	auto last  = m_init_vec.end() - 1;
	auto first = m_init_vec.end() - 2;

	double dtime = last->time - first->time;
	
	if ( (fabs(dtime) < COMMON_EPS) || (dtime > 1e2) ) {
		return false;
	}
	
	*a_dtime = dtime;
	return true;
}

bool initializer_by_points::calculate_acceleration(double* a_acceleration) {
	auto last   = m_init_vec.end() - 1;
	auto middle = m_init_vec.end() - 2;
	auto first  = m_init_vec.end() - 3;
	
	double current_time = last->time - middle->time;
	double current_speed = 
		(last->cos_v - middle->cos_v) / current_time;
	
	double initial_time = middle->time - first->time;
	double initial_speed = 
		(middle->cos_v - first->cos_v) / initial_time;
		
	double current_acceleration = 
		(current_speed - initial_speed) / current_time;
	
	if ( fabs(current_acceleration) > 1e2 ) {
		return false;
	}
	
	*a_acceleration = current_acceleration;
	return true;
}
