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

include(FetchContent)
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.8
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw OpenGL::GL)
target_compile_options(imgui_lib PRIVATE -w)

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
    imgui_lib
)
