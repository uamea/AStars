cmake_minimum_required(VERSION 3.16)
project(Astars)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ncurses library
find_package(PkgConfig REQUIRED)
pkg_check_modules(NCURSES REQUIRED ncurses)

# Include include directories
include_directories(
    include/
    include/algorithms/
)

# Create executable
add_executable(astars 
    src/main.cpp 
    src/astar_viz.cpp 
    src/astar_app.cpp 
    src/algorithms/algorithm_utils.cpp)

# Link ncurses library
target_link_libraries(astars ${NCURSES_LIBRARIES})
target_include_directories(astars PRIVATE ${NCURSES_INCLUDE_DIRS})
target_compile_options(astars PRIVATE ${NCURSES_CFLAGS_OTHER})
