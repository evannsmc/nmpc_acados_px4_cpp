cmake_minimum_required(VERSION 3.14)
project(nmpc_acados_px4_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(quad_platforms_cpp REQUIRED)

# autodiff: try installed version first, otherwise fetch
find_package(autodiff QUIET)
if(NOT autodiff_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    autodiff
    GIT_REPOSITORY https://github.com/autodiff/autodiff.git
    GIT_TAG v1.1.2
  )
  set(AUTODIFF_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(AUTODIFF_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
  set(AUTODIFF_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(AUTODIFF_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(autodiff)
endif()

find_package(quad_trajectories_cpp REQUIRED)
find_package(ros2_logger_cpp REQUIRED)

# ---------------------------------------------------------------------------
# Acados paths
# ---------------------------------------------------------------------------
set(ACADOS_ROOT
    "/home/egmc/acados"
    CACHE PATH "Acados installation root (contains include/ and lib/)")

# Path to the generated solver directory.
# The Python package must be run FIRST to generate these files.
#   ros2 run nmpc_acados_px4 run_node --platform sim --trajectory hover --hover-mode 1
set(GENERATED_SOLVER_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../nmpc_acados_px4/nmpc_acados_px4_utils/controller/nmpc/acados_generated_files/holybro_euler_err_mpc_c_generated_code"
    CACHE PATH "Directory produced by acados code generation (Python package)")

# Fail early with a helpful message if the generated code is missing
if(NOT EXISTS "${GENERATED_SOLVER_DIR}/acados_solver_holybro_euler_err.h")
  message(FATAL_ERROR
    "\n[nmpc_acados_px4_cpp] Generated solver header not found at:\n"
    "  ${GENERATED_SOLVER_DIR}/acados_solver_holybro_euler_err.h\n\n"
    "Run the Python package first to generate the acados C solver:\n"
    "  ros2 run nmpc_acados_px4 run_node --platform sim --trajectory hover --hover-mode 1\n"
    "Then rebuild this package.\n")
endif()

# ---------------------------------------------------------------------------
# Main executable
# ---------------------------------------------------------------------------
add_executable(run_node
    src/nmpc_solver.cpp
    src/offboard_control_node.cpp
    src/run_node.cpp
)

target_include_directories(run_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    # Acados public headers
    ${ACADOS_ROOT}/include
    ${ACADOS_ROOT}/include/blasfeo/include
    ${ACADOS_ROOT}/include/hpipm/include
    # Generated solver header (acados_solver_holybro_euler_err.h + model headers)
    ${GENERATED_SOLVER_DIR}
    # ROS2 headers
    ${rclcpp_INCLUDE_DIRS}
    ${px4_msgs_INCLUDE_DIRS}
)

target_link_libraries(run_node PRIVATE
    quad_platforms_cpp::quad_platforms_cpp
    quad_trajectories_cpp::quad_trajectories_cpp
    Eigen3::Eigen
    ros2_logger_cpp::ros2_logger_cpp
    ${rclcpp_LIBRARIES}
    ${px4_msgs_LIBRARIES}
    # Acados runtime libraries
    ${ACADOS_ROOT}/lib/libacados.so
    ${ACADOS_ROOT}/lib/libblasfeo.so
    ${ACADOS_ROOT}/lib/libhpipm.so
    # Generated solver shared library (compiled by the Python build step)
    ${GENERATED_SOLVER_DIR}/libacados_ocp_solver_holybro_euler_err.so
)

# Set RPATH so the installed binary finds the shared libraries at runtime
set_target_properties(run_node PROPERTIES
    INSTALL_RPATH
        "${ACADOS_ROOT}/lib;${GENERATED_SOLVER_DIR}"
    BUILD_RPATH
        "${ACADOS_ROOT}/lib;${GENERATED_SOLVER_DIR}"
)

install(TARGETS run_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
