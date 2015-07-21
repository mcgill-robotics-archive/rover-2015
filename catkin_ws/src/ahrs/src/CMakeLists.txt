add_subdirectory(libs)

add_library(common STATIC
    libs/src/PeriodicThread.cpp
    libs/include/common/PeriodicThread.h
)

target_link_libraries(common
                      ${Boost_THREAD_LIBRARIES}
                      ${Boost_SYSTEM_LIBRARIES}
                      )

file(GLOB AHRS_NODE_SOURCES [^.]*.cpp)
file(GLOB AHRS_NODE_PRIVATE_HEADERS [^.]*.h)

add_executable(ahrs_node
    ${AHRS_NODE_SOURCES}
    ${AHRS_NODE_PRIVATE_HEADERS}
)

target_link_libraries(ahrs_node
${catkin_LIBRARIES}
ahrs
)

add_dependencies(ahrs_node rover_msgs_generate_messages_cpp)
