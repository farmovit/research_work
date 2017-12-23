#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <getopt.h>
#include <memory>
#include <string>
#include <sstream>
#include <fstream>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "XML_Configurator.h"
#include "tracking.h"
#include "identifier.h"
#include "identifier_checker.h"
#include "pugixml.hpp"
#include "pugi_tools.h"
#include "tracking_object_factory.h"

void load_csv(std::istream& is, measurement_data& data)
{
	std::string tmp;
	
	if(!getline(is, tmp, ';'))
	{
		return;
	}
	
	data.time = std::stof(tmp);
	
	getline(is, tmp, ';');
	data.cos_v = std::stof(tmp);
	
	getline(is, tmp);
	data.type = std::stoi(tmp);
}

std::istream& operator>> (std::istream& is, measurement_data& data)
{	
	load_csv(is, data);
	return is;
}

void save_objects_info(const tracking_object_storage& a_object_list, const fs::path& output_file )
{
	std::ofstream ofs(output_file.string());
	if (!ofs.is_open()) 
	{ 
		std::stringstream err;
		err << "Cannot open " << output_file << std::endl;
		throw std::runtime_error(err.str());
	}
	
	ofs << std::fixed << std::setprecision(6) << std::setw(6);
	
	auto end_it = a_object_list.end();
	for(auto it = a_object_list.begin(); it != end_it; ++it) 
	{
		ofs << "Time;OBJ_1" << std::endl;
		auto object = *it;
		auto params = object->get_params();
		std::size_t angles_count = params.v_angles.size();
		std::size_t time_count = params.v_angles.size();
		if( angles_count != time_count ) 
		{
			std::stringstream err;
			err << "The number of angles != the number of times: " << angles_count << "!=" << time_count << std::endl;
			throw std::runtime_error(err.str());
		}
		for(auto i = 0; i < angles_count; ++i) 
		{
			ofs << params.v_times[i] << ";" << params.v_angles[i] << std::endl;
		}
	}
	
	ofs.close();
}

void process_trajectory(const fs::path& input_file, const fs::path& output_tracking_file, std::shared_ptr<tracking_object_factory> object_factory, std::shared_ptr<identifier> a_identifier) 
{
	if(!fs::is_regular_file(input_file)) 
	{
		std::stringstream err;
		err << input_file << " is not a regular file!" << std::endl;
		throw std::runtime_error(err.str());
	}
	
	std::ifstream ifs(input_file.string());
	if (!ifs.is_open()) {
		std::stringstream err;
		err << "Cannot open file: " << input_file << std::endl;
		throw std::runtime_error(err.str());
	}
	std::cout << "File_opened: " << input_file << std::endl;
	
	std::string head;
	getline(ifs, head);
	
	tracking track(a_identifier, object_factory);
	
	measurement_data tracking_data;
	while(ifs >> tracking_data) 
	{
		track.delete_old(tracking_data.time);
		track.go(tracking_data);
	}
	
	auto tracked_object_list = track.get_trecked_object_list();
	
	save_objects_info(tracked_object_list, output_tracking_file);
}

void process_trajectories(const fs::path &input_path, const fs::path &output_path, std::shared_ptr<tracking_object_factory> object_factory, std::shared_ptr<identifier> a_identifier )
{
	for (fs::directory_iterator it(input_path), end; it != end; ++it) 
	{
		if (it->path().extension() == ".csv") 
		{
			std::cout << "INFO: Processing file: " << it->path() << std::endl;
			std::string filename = "tracking_";
			fs::path trajectory_output_file = output_path / (filename + fs::basename(it->path()) + ".csv");
			process_trajectory(it->path(), trajectory_output_file, object_factory, a_identifier);
		}
	}
}

int main(int argc, char** argv) 
try {
	fs::path trajectory_input;
	fs::path trajectory_output;
	fs::path property_file;
	
	int c;
  while((c = getopt(argc, argv, "i:o:p:")) != -1) 
	{
    switch (c) 
		{
      case 'i':
        trajectory_input = optarg;
        break;
			
			case 'o':
				trajectory_output = optarg;
				break;
			
			case 'p':
				property_file = optarg;
				break;
			
			default:
				throw std::runtime_error("Unrecognized option");
		}
	}
	
	pugi::xml_document xml_doc;
	property_file = ( !fs::exists(property_file) ) ? "tracking_property.xml" : property_file;
	
	XML_Configurator tracking_object_configurator;
	tracking_object_configurator.parse(property_file.string());
	std::shared_ptr<tracking_object_factory_creator> tracking_object_creator = tracking_object_configurator.get_tracking_object_factory_creator();
	std::shared_ptr<tracking_object_factory> object_factory = tracking_object_creator->get_factory();
	
	std::shared_ptr<identifier> identifier_ = tracking_object_configurator.get_identifier();

	if(fs::is_regular_file(trajectory_input))
	{
		fs::path trajectory_output_file = trajectory_output / "tracking.csv";
		process_trajectory(trajectory_input, trajectory_output_file, object_factory, identifier_);
	}
	else if(fs::exists(trajectory_input))
	{
		process_trajectories(trajectory_input, trajectory_output, object_factory, identifier_);
	}
	else 
	{
		std::stringstream err;
		err << trajectory_input << " does not exist!" << std::endl;
		throw std::runtime_error(err.str());
	}
	
	return 0;
}
catch (std::exception& e) {
  std::cerr << "ERROR: " << e.what() << std::endl;
	return 1;
}
catch (...) {
	std::cerr << "Undefined error" << std::endl;
	return 2;
}