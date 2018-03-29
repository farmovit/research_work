#pragma once

#include <memory>

#include "tracking_object.h"
#include "tracking_object_factory.h"
#include "structions.h"
#include "identifier.h"


const float C_SIGMA_PEL = 100000;
enum {C_MIN_PEL_OUT_NUM = 1001, C_MAX_PEL_OUT_NUM = 0xffff};

//TODO: think about it:
const int C_DELTA_T_OLD = 50;

class tracking
{
public:
	tracking(std::shared_ptr<identifier>, std::shared_ptr<tracking_object_factory> );
	~tracking();
	
	tracking_object_storage get_trecked_object_list() const;
	tracking_object get_tracked_object() const;
	
	void reset();
	void go(const measurement_data& );
	void delete_old(const double& );
	
private:
	std::size_t get_new_number();
	void add_new_object(const measurement_data& );
	void set_current_object(const tracking_object& );
	
	tracking_object_storage m_tracked_object_list;
	std::shared_ptr<identifier> m_identifier;
	std::shared_ptr<tracking_object_factory> m_tracking_object_factory;
	
	tracking_object m_current_tracked_object;
	
	std::size_t m_out_number;
};
