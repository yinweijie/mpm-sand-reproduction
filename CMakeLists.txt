cmake_minimum_required(VERSION 3.21)

project(klar2016_sand LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(GTest REQUIRED)

add_library(klar2016_core
    src/core/simulation.cpp
    src/core/simulation_config.cpp
    src/core/simulation_detail.cpp
    src/core/simulation_io.cpp
    src/core/simulation_particles.cpp
    src/core/simulation_step.cpp
)

target_include_directories(klar2016_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(klar2016_core
    PUBLIC
        Eigen3::Eigen
        fmt::fmt
        nlohmann_json::nlohmann_json
)

target_compile_options(klar2016_core
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

add_executable(${PROJECT_NAME}
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        klar2016_core
)

include(CTest)

if(BUILD_TESTING)
    add_executable(klar2016_smoke
        tests/smoke_test.cpp
    )

    target_link_libraries(klar2016_smoke
        PRIVATE
            klar2016_core
            GTest::gtest_main
    )

    target_compile_definitions(klar2016_smoke
        PRIVATE
            KLAR2016_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )

    target_compile_options(klar2016_smoke
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )

    include(GoogleTest)
    gtest_discover_tests(klar2016_smoke)
endif()
