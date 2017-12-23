cmake_minimum_required(VERSION 3.0)

set(project_name "tracking_test")

project(${project_name})

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS 
		filesystem
		system
	REQUIRED
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen3")
set(PUGIXML_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pugi_xml")

include_directories(
	"tracking_lib/include"
	${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${PUGIXML_SRC_DIR}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden -O2 -Wa,-mbig-obj -std=c++14" )

add_executable(${project_name} "main.cpp")

add_subdirectory(tracking_lib)

target_link_libraries(${project_name} trackinglib ${Boost_LIBRARIES})
