#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
include_directories(..)

if(NOT GTEST_LIBRARY)
   message("GTest library not found")
endif()

enable_testing()
###################################

add_executable(utest utest.cpp)
# Linking to gtest and to your libraries
target_link_libraries(utest GTest::GTest GTest::Main shapes)
# We can finally add the test to a list of tests to be run with make test
add_test(shapes_test utest)
