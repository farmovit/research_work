#include "identifier.h"
#include "tracking_object.h"

#include <memory>
#include <iostream>

#define _DEBUG_IDENTIFIER 0

identifier::identifier(): m_checker(nullptr)
{
}

identifier::~identifier()
{
}

void identifier::set_checker(std::shared_ptr<identification_checker_base> a_checker)
{
  m_checker = a_checker;
}

bool identifier::check(const tracking_object &object, 
											 const measurement_data &data)
{
  if(m_checker == nullptr) {
		return true;
  } else {
		return m_checker->check(object, data);
	}
}

identified_object_storage_ptr identifier::identify(
									const tracking_object_storage& l_track_obj,
									const measurement_data& data) 
{
	identified_object_storage_ptr identified_objects = std::make_shared<identified_object_storage>();
  
  auto last = l_track_obj.end();

  for(auto it = l_track_obj.begin(); it != last; ++it)
  {
    if(check((*it->get()), data))
    {
#if _DEBUG_IDENTIFIER
			std::cout << "Object is identified" << std::endl;
#endif
      identified_object_data id(*it);
      identified_objects->push_back(id);
    } 
		else
		{
#if _DEBUG_IDENTIFIER
			std::cout << "Cannot identify object" << std::endl;
			std::cout << "Data: " << data.time << " " << data.cos_v << " " << data.type << std::endl;
#endif
			if((*it)->is_initialized()) {
				(*it)->extrapolate(data);
			}
		}
  }

  return identified_objects;
}

