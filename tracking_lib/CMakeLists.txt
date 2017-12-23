set(project_name "trackinglib")

project(${project_name})

include_directories(
	"include"
	${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${PUGIXML_SRC_DIR}
)

set(SOURCES 
	"src/extrapolator.cpp"
	"src/extrapolator_factory.cpp"
	"src/filter.cpp"
	"src/filter_factory.cpp"
	"src/identifier.cpp"
	"src/identifier_checker.cpp"
	"src/initializer.cpp"
	"src/initializer_factory.cpp"
	"src/tracking_object.cpp"
	"src/tracking.cpp"
	"src/pugi_tools.cpp"
	"src/tracking_object_factory.cpp"
	"src/XML_Configurator.cpp"
	"${PUGIXML_SRC_DIR}/pugixml.cpp"
)

set(HEADERS
	"include/extrapolator.h"
	"include/extrapolator_factory.h"
	"include/filter.h"
	"include/filter_factory.h"
	"include/identifier.h"
	"include/identifier_checker.h"
	"include/initializer.h"
	"include/initializer_factory.h"
	"include/wrapped_factory.h"
	"include/structions.h"
	"include/tracking_object.h"
	"include/tracking.h"
	"include/pugi_tools.h"
	"include/tracking_object_factory.h"
	"include/factory_creator.h"
	"include/XML_Configurator.h"
)

add_library(${project_name} ${SOURCES} ${HEADERS})