set(ifdeftest_SRCS ifdeftest.c ifdeftest.h)
add_definitions(-DSLAVE) # same as #define SLAVE in .c
add_executable(ifdef ${ifdeftest_SRCS})