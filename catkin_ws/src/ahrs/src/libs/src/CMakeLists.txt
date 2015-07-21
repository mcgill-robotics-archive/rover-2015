include_directories(${COM_HEADER})
include_directories(${SBG_BASE})

file(GLOB AHRS_SOURCES [^.]*.cpp)
file(GLOB AHRS_PRIVATE_HEADERS [^.]*.h)
file(GLOB AHRS_PUBLIC_HEADERS ../include/ahrs/[^.]*.h)

add_library(ahrs STATIC
            ${AHRS_SOURCES}
            ${AHRS_PRIVATE_HEADERS}
            ${AHRS_PUBLIC_HEADERS}
            ${Boost_THREAD_LIBRARIES}
            ${Boost_SYSTEM_LIBRARIES})

target_link_libraries(ahrs sbgCom common)
