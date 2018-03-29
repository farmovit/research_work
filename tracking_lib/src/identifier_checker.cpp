#include <cmath>
#include "identifier_checker.h"

#define _DEBUG_CHECERS 0

identification_checker_base::~identification_checker_base() {}

identification_checker_residual::identification_checker_residual(double a_limit) : m_limit(a_limit) {}

identification_checker_residual::~identification_checker_residual() {}
  
bool identification_checker_residual::check_self (
        const tracking_object &object,
        const measurement_data &data)
{
    tracking_object_params params;
    bool flag;
    params = object.get_params();
    if(fabs(params.angle - data.cos_v) <= m_limit) flag = true;
    else                                           flag = false;

#if _DEBUG_CHECERS
    std::cout << "residual checker: " << params.angle << " " << data.cos_v << " " << m_limit << " " << flag << std::endl;
#endif  
    return flag;
}

identification_checker_type::identification_checker_type() {}
identification_checker_type::~identification_checker_type() {}

bool identification_checker_type::check_self(
        const tracking_object &object,
		const measurement_data &data)
{
    bool flag;
	
    if(object.get_type() == data.type) flag = true;
    else                               flag = false;

#if _DEBUG_CHECERS  
    std::cout << "type checker: " << flag << std::endl;
#endif
	
	return flag;
}

identification_checker_alive::identification_checker_alive() {}
identification_checker_alive::~identification_checker_alive() {}
  
bool identification_checker_alive::check_self(
        const tracking_object &object,
		const measurement_data &data)
{
#if _DEBUG_CHECERS  
    std::cout << "alive checker: " << object.is_alive() << std::endl;
#endif
	
    return object.is_alive();
}
