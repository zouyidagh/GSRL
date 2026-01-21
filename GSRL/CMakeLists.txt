cmake_minimum_required(VERSION 3.22)

project(gsrl LANGUAGES C CXX)

add_library(${PROJECT_NAME} OBJECT)

# Add CMSIS-DSP library
set(CMSISCORE "${CMAKE_SOURCE_DIR}/CubeMX_BSP/Drivers/CMSIS/Core")
add_subdirectory(Dependence/CMSIS-DSP)

# Add Eigen library
add_subdirectory(Dependence/eigen)

# Add OSQP library
set(EMBEDDED 2)
set(OSQP_EMBEDDED_MODE 2)
set(OSQP_USE_FLOAT ON)
add_subdirectory(Dependence/osqp)
target_compile_options(OSQPLIB PRIVATE -w)
target_compile_options(osqpstatic PRIVATE -w)

file(GLOB_RECURSE GSRL_ALGORITHM_SOURCES
    "Algorithm/src/*.c"
    "Algorithm/src/*.cpp"
)

file(GLOB_RECURSE GSRL_DEVICE_SOURCES
    "Device/src/*.c"
    "Device/src/*.cpp"
)

file(GLOB_RECURSE GSRL_DRIVER_SOURCES
    "Driver/src/*.c"
    "Driver/src/*.cpp"
)

target_sources(${PROJECT_NAME} PRIVATE
    ${GSRL_ALGORITHM_SOURCES}
    ${GSRL_DEVICE_SOURCES}
    ${GSRL_DRIVER_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    Include
    Algorithm/inc
    Device/inc
    Driver/inc
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    EIGEN_NO_MALLOC
)

target_compile_features(${PROJECT_NAME} PUBLIC
    c_std_11
    cxx_std_17
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
    stm32cubemx
    CMSISDSP
    eigen
    osqpstatic
)
