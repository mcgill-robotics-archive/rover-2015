add_subdirectory(lib)

add_library(common STATIC
    lib/src/PeriodicThread.cpp
    lib/include/common/PeriodicThread.h
)

target_link_libraries(common
                      ${Boost_THREAD_LIBRARIES}
                      ${Boost_SYSTEM_LIBRARIES}
                      )

#file(GLOB AHRS_NODE_SOURCES [^.]*.cpp)
#file(GLOB AHRS_NODE_PRIVATE_HEADERS [^.]*.h)
#
#add_executable(ahrs_node
#    ${AHRS_NODE_SOURCES}
#    ${AHRS_NODE_PRIVATE_HEADERS}
#)