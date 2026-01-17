# generate-config.cmake - 自动生成配置文件CMake脚本

# 操作系统检测
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
    set(CONFIG_OS "Windows")
    set(CONFIG_PATH_SEP "\\")
    set(CONFIG_EXE_SUFFIX ".exe")
elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
    set(CONFIG_OS "Linux")
    set(CONFIG_PATH_SEP "/")
    set(CONFIG_EXE_SUFFIX "")
elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
    set(CONFIG_OS "macOS")
    set(CONFIG_PATH_SEP "/")
    set(CONFIG_EXE_SUFFIX "")
else()
    set(CONFIG_OS "Unknown")
    set(CONFIG_PATH_SEP "/")
    set(CONFIG_EXE_SUFFIX "")
endif()

# 工程信息
if(NOT DEFINED PROJECT_NAME OR PROJECT_NAME STREQUAL "")
    set(CONFIG_PROJECT_NAME "GMStdRobotLib")
else()
    set(CONFIG_PROJECT_NAME ${PROJECT_NAME})
endif()

if(NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CONFIG_BUILD_TYPE "Debug")
else()
    set(CONFIG_BUILD_TYPE ${CMAKE_BUILD_TYPE})
endif()

set(CONFIG_ELF_PATH_CMAKE "build/${CONFIG_BUILD_TYPE}/${CONFIG_PROJECT_NAME}.elf")
set(CONFIG_ELF_PATH_EIDE "build/${CONFIG_PROJECT_NAME}/${CONFIG_PROJECT_NAME}.elf")
set(CONFIG_COMPILE_COMMANDS_DIR "build/${CONFIG_BUILD_TYPE}")
set(CONFIG_TOOLCHAIN_PREFIX "arm-none-eabi-")
set(CONFIG_TOOLCHAIN_PREFIX_NO_DASH "arm-none-eabi")
set(CONFIG_OPENOCD_INTERFACE "cmsis-dap.cfg")
set(CONFIG_OPENOCD_TARGET "stm32f4x.cfg")

# 工具链路径检测
find_program(ARM_GCC_PATH "${CONFIG_TOOLCHAIN_PREFIX}gcc")
if(ARM_GCC_PATH)
    get_filename_component(CONFIG_TOOLCHAIN_BIN_DIR ${ARM_GCC_PATH} DIRECTORY)
    if(CONFIG_OS STREQUAL "Windows")
        string(REPLACE "/" "\\" CONFIG_TOOLCHAIN_BIN_DIR_NATIVE "${CONFIG_TOOLCHAIN_BIN_DIR}")
        set(CONFIG_QUERY_DRIVER "${CONFIG_TOOLCHAIN_BIN_DIR_NATIVE}\\${CONFIG_TOOLCHAIN_PREFIX}*")
    else()
        set(CONFIG_QUERY_DRIVER "${CONFIG_TOOLCHAIN_BIN_DIR}/${CONFIG_TOOLCHAIN_PREFIX}*")
    endif()
else()
    if(CONFIG_OS STREQUAL "Windows")
        set(CONFIG_QUERY_DRIVER "D:\\Coding\\Envs\\Compiler\\arm-gnu\\bin\\${CONFIG_TOOLCHAIN_PREFIX}*")
    else()
        set(CONFIG_QUERY_DRIVER "/usr/bin/${CONFIG_TOOLCHAIN_PREFIX}*")
    endif()
endif()

# OpenOCD 配置
set(CONFIG_OPENOCD_INTERFACE "cmsis-dap.cfg")
set(CONFIG_OPENOCD_TARGET "stm32f4x.cfg")
find_program(OPENOCD_PATH "openocd")

# 模板和输出目录
set(CONFIG_TEMPLATE_DIR "${CMAKE_SOURCE_DIR}/cmake/templates")
set(CONFIG_VSCODE_DIR "${CMAKE_SOURCE_DIR}/.vscode")

if(NOT EXISTS "${CONFIG_VSCODE_DIR}")
    file(MAKE_DIRECTORY "${CONFIG_VSCODE_DIR}")
endif()

# 配置文件生成函数
function(generate_config_file TEMPLATE_FILE OUTPUT_FILE DESCRIPTION)
    set(TEMPLATE_PATH "${CONFIG_TEMPLATE_DIR}/${TEMPLATE_FILE}")
    
    if(NOT EXISTS "${TEMPLATE_PATH}")
        message(WARNING "[config] Template not found: ${TEMPLATE_PATH}")
        return()
    endif()
    
    if(EXISTS "${OUTPUT_FILE}")
        return()
    endif()
    
    configure_file("${TEMPLATE_PATH}" "${OUTPUT_FILE}" @ONLY)
    message(STATUS "[config] Generated: ${OUTPUT_FILE}")
endfunction()

# 生成配置文件
generate_config_file("launch.json.in" "${CONFIG_VSCODE_DIR}/launch.json" "launch.json")
generate_config_file("settings.json.in" "${CONFIG_VSCODE_DIR}/settings.json" "settings.json")
generate_config_file("flash.cfg.in" "${CMAKE_SOURCE_DIR}/flash.cfg" "flash.cfg")
generate_config_file(".lazy.lua.in" "${CMAKE_SOURCE_DIR}/.lazy.lua" ".lazy.lua")
