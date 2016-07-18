project(elm_example)
cmake_minimum_required(VERSION 2.8)

find_package(Eigen3 REQUIRED)

set(Boost_USE_STATIC_LIBS        ON) # only find static libs
set(Boost_USE_MULTITHREADED      ON)
find_package(Boost REQUIRED COMPONENTS date_time)


include_directories(${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

aux_source_directory(. SRC_LIST)


add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

