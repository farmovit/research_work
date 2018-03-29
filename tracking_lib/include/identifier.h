#pragma once

#include <memory>

#include "structions.h"
#include "tracking_object.h"
#include "identifier_checker.h"

struct identified_object_data
{
	tracking_object_ptr m_tracking_object;

	identified_object_data() {};
	
  	identified_object_data(tracking_object_ptr a_tracking_object):
      	m_tracking_object(a_tracking_object) {}
};

typedef std::shared_ptr<identified_object_data> identified_object_data_ptr;
typedef std::list<identified_object_data> identified_object_storage;
typedef identified_object_storage::iterator identified_object_iterator;
typedef std::shared_ptr<identified_object_storage> identified_object_storage_ptr;

class identifier 
{
public:	
	identified_object_storage_ptr identify(const tracking_object_storage&,
										   const measurement_data& );
							
	void set_checker(std::shared_ptr<identification_checker_base> a_checker);

	identifier();
	~identifier();

private:
  	std::shared_ptr<identification_checker_base> m_checker;

  	bool check(const tracking_object &object, const measurement_data &data);
};
