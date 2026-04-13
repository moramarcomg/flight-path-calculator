cmake_minimum_required(VERSION 3.10)
project(FlightPathCalculator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTS "Build unit tests" ON)

include_directories(${PROJECT_SOURCE_DIR}/include)

set(SOURCES
    src/flight_path_calculator.cpp
    src/main.cpp
)

add_executable(flight_path_calculator ${SOURCES})

target_include_directories(flight_path_calculator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(BUILD_TESTS)
    enable_testing()
    
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    
    FetchContent_MakeAvailable(googletest)
    
    set(TEST_SOURCES
        tests/test_flight_path.cpp
        src/flight_path_calculator.cpp
    )
    
    add_executable(flight_path_tests ${TEST_SOURCES})
    target_link_libraries(flight_path_tests gtest_main gtest)
    
    target_include_directories(flight_path_tests PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    
    add_test(NAME FlightPathTests COMMAND flight_path_tests)
endif()
