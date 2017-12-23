#include "tracking_object.h"

#include <iostream>

#define __DEBUG_TRACKING 1

void tracking_object::update(const measurement_data &data)
{ 
  if(!m_initializer->check_state()) 
  {
    if (!m_initializer->initialize(data)) 
    {
			filter_params temp = m_initializer->get_filter_param();
			set_data(temp);
#if __DEBUG_TRACKING
			std::cout << "INFO: Initializing is failed\n";
#endif
      return;
    }
    
    if (m_initializer->check_state())
    { 
#if __DEBUG_TRACKING
			std::cout << "INFO: Bad state\n";
#endif
      filter_params temp = m_initializer->get_filter_param();
			m_filter->initialize(temp);
			m_extrapolator->put(temp);
			
			set_data(temp);
    }
    
    return;
  }

#if __DEBUG_TRACKING
	std::cout << "INFO: Filtrating...\n";
#endif

  if(!m_filter->filtrate(data)) 
	{
#if __DEBUG_TRACKING
		std::cout << "INFO: Filtrating is failed\n";
#endif
		return;
	}
		
  filter_params temp = m_filter->get_filtrated_data();
	m_extrapolator->put(temp);
	
	set_data(temp);
}

void tracking_object::extrapolate(const measurement_data& measurement) {
	filter_params extrapolated_data =
		m_extrapolator->extrapolate(measurement.time);
	
	set_data(extrapolated_data);
}

void tracking_object::set_data(const filter_params& a_data) 
{
	m_params.delta_time 	= a_data.delta_time;
	m_params.time_last 		= a_data.time_last;
	m_params.angle 				= a_data.angle;
	m_params.speed 				= a_data.speed;
	m_params.acceleration = a_data.acceleration;
	m_params.v_angles.push_back(a_data.angle);
	m_params.v_times.push_back(a_data.time_last);
}

void tracking_object::set_type(const int& a_type)
{
  m_params.type = a_type;
}

void tracking_object::set_number(const int& a_number)
{
  m_params.internal_idx = a_number;
}

void tracking_object::set_out_number(const int& a_numb)
{
  m_params.out_idx = a_numb;
}

void tracking_object::set_time(const double& a_time) {
		m_params.time_last = a_time;
}

int tracking_object::get_number() const {
	return m_params.internal_idx;
}

int tracking_object::get_out_number() const {
	return m_params.out_idx;
}

bool tracking_object::is_alive() const
{
  return m_params.alive;
}

bool tracking_object::is_initialized() const
{
  return m_initializer->check_state();
}

int tracking_object::get_type() const
{
  return m_params.type;
}

double tracking_object::get_time() const
{
	return m_params.time_last;
}

double tracking_object::get_angle() const
{
	return m_params.angle;
}

tracking_object_params tracking_object::get_params() const
{
	return m_params;
}

void tracking_object::kill()
{
  m_params.alive = false;
}


void tracking_object::set_initializer(std::shared_ptr<abstract_initializer> a_initializer)
{
  m_initializer = a_initializer;
}

void tracking_object::set_filter(std::shared_ptr<abstract_filter> a_filter)
{
  m_filter = a_filter;
}

void tracking_object::set_extrapolator(std::shared_ptr<abstract_extrapolator> a_extrapolator)
{
  m_extrapolator = a_extrapolator;
}
