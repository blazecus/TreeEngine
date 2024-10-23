cmake_minimum_required(VERSION 3.1...3.25)
project(
	Engine
	VERSION 0.1.0
	LANGUAGES CXX C
)

include(utils.cmake)

#set(CMAKE_C_COMPILER cl.exe)
#set(CMAKE_CXX_COMPILER cl.exe)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# We add an option to enable different settings when developping the app than
# when distributing it.
option(DEV_MODE "Set up development helper settings" ON)

add_subdirectory(webgpu)
add_subdirectory(glfw3webgpu)
add_subdirectory(imgui)

add_executable(App
	main.cpp
	Application.h
	Application.cpp
  ClothObject.h
  ClothObject.cpp
  TreeGenerator.h
  TreeGenerator.cpp
	ResourceManager.h
	ResourceManager.cpp
	implementations.cpp
)

if(DEV_MODE)
	# In dev mode, we load resources from the source tree, so that when we
	# dynamically edit resources (like shaders), these are correctly
	# versionned.
	target_compile_definitions(App PRIVATE
		RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources"
	)
else()
	# In release mode, we just load resources relatively to wherever the
	# executable is launched from, so that the binary is portable
	target_compile_definitions(App PRIVATE
		RESOURCE_DIR="./resources"
	)
endif()

target_include_directories(App PRIVATE .)

target_link_libraries(App PRIVATE glfw webgpu glfw3webgpu imgui)

set_target_properties(App PROPERTIES
	CXX_STANDARD 17
	VS_DEBUGGER_ENVIRONMENT "DAWN_DEBUG_BREAK_ON_ERROR=1"
)
target_treat_all_warnings_as_errors(App)
target_copy_webgpu_binaries(App)

if (MSVC)
	# Ignore a warning that GLM requires to bypass
	# Disable warning C4201: nonstandard extension used: nameless struct/union
	target_compile_options(App PUBLIC /wd4201)
	# Disable warning C4305: truncation from 'int' to 'bool' in 'if' condition
	target_compile_options(App PUBLIC /wd4305)

	# Ignore a warning that stb_image requires to bypass
	# Disable warning C4244: conversion from 'int' to 'short', possible loss of data
	target_compile_options(App PUBLIC /wd4244)
endif (MSVC)

#add_subdirectory(glfw)
# At the end of the CMakeLists.txt
if (EMSCRIPTEN)
    # Add Emscripten-specific link options
    target_link_options(App PRIVATE
        -sUSE_WEBGPU # Handle WebGPU symbols
        -sASYNCIFY # Required by WebGPU-C++
    	--preload-file "${CMAKE_CURRENT_SOURCE_DIR}/resources"
		-sALLOW_MEMORY_GROWTH
    )
    # Emscripten has built-in support for GLFW but requires the `-sUSE_GLFW=3` link option:
    add_library(glfw INTERFACE)
    target_link_options(glfw INTERFACE -sUSE_GLFW=3)
	set_target_properties(App PROPERTIES SUFFIX ".html")
else()
    add_subdirectory(glfw)
endif()