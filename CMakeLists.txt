cmake_minimum_required(VERSION 3.16)

project(physics_simulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(MSVC)
    set(PHYSICS_SIMULATION_WARNING_FLAGS
        /W4
        /permissive-
    )
else()
    set(PHYSICS_SIMULATION_WARNING_FLAGS
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
    )
endif()

find_package(OpenGL REQUIRED)
find_package(Freetype REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(physics_simulation
    src/main.cpp
    src/physics/contact_solver.cpp
    src/physics/physics_world.cpp
    src/physics/rigid_body.cpp
    src/render/debug_renderer.cpp
    src/render/font_renderer.cpp
)

target_include_directories(physics_simulation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_options(physics_simulation PRIVATE
    ${PHYSICS_SIMULATION_WARNING_FLAGS}
)

target_link_libraries(physics_simulation PRIVATE
    OpenGL::GL
    Freetype::Freetype
    glfw
)
