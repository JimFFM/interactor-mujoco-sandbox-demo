cmake_minimum_required(VERSION 3.16)
project(mjtests CXX C)

get_filename_component(DEMO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(MJ "${DEMO_ROOT}/third_party/riscv64-mujoco")

set(_runtime_api "${DEMO_ROOT}/../../2-contract/guest-runtime/vendor/sandbox-api")
if(EXISTS "${_runtime_api}/docker/api/api.cpp")
	set(SANDBOX_API "${_runtime_api}" CACHE PATH "godot-sandbox guest API checkout")
else()
	set(SANDBOX_API "${DEMO_ROOT}/third_party/sandbox-api" CACHE PATH "godot-sandbox guest API checkout")
endif()
set(DOWNLOAD_RUNTIME_API OFF CACHE BOOL "" FORCE)
add_subdirectory("${SANDBOX_API}/cmake" sandbox_cmake)

# The suite runs inside the emulator, because that is where the defects it
# looks for live. Nothing here runs on the host.
add_sandbox_program(mjtests
	main.cpp
	test_strings.cpp
	test_alloc.cpp
	test_xml.cpp
	test_wrapped_libc.cpp
	test_frames.cpp
	test_station.cpp
	"${DEMO_ROOT}/guest/station.cpp")

target_include_directories(mjtests PRIVATE
	"${MJ}/include"
	"${DEMO_ROOT}/third_party/witness-cpp/include"
	"${DEMO_ROOT}/third_party/doctest"
	"${DEMO_ROOT}/guest")

target_link_libraries(mjtests PRIVATE
	"${MJ}/lib/libmujoco.a" "${MJ}/lib/libccd.a" "${MJ}/lib/libtinyxml2.a"
	"${MJ}/lib/libqhullstatic_r.a" "${MJ}/lib/libtinyobjloader.a" "${MJ}/lib/libminiz.a" "${MJ}/lib/liblodepng.a"
	-lm -lstdc++)
