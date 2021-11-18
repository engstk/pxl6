cmake_minimum_required(VERSION 3.13)

project(mac)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../tools/cmake)

option(ENABLE_TEST_COVERAGE "Enable test coverage" OFF)
option(ENABLE_MAC_UTEST "Enable mac utest" OFF)

enable_testing()

add_library(mac
	ca.c
	default_region.c
	endless_scheduler.c
	nfcc_coex_session.c
	nfcc_coex_access.c
	nfcc_coex_region.c
	nfcc_coex_region_call.c
	fira_access.c
	fira_crypto.c
	fira_round_hopping_sequence.c
	fira_frame.c
	fira_region.c
	fira_region_call.c
	fira_session.c
	fproc.c
	fproc_broken.c
	fproc_multi.c
	fproc_vendor.c
	fproc_nothing.c
	fproc_rx.c
	fproc_stopped.c
	fproc_tx.c
	frame.c
	ie.c
	mcps_main.c
	ops.c
	schedule.c
	schedulers.c
	simple_ranging_region.c
	regions.c
	utils_fixed_point.c)
target_include_directories(mac PUBLIC include)
target_compile_options(mac PRIVATE
	-Wall
	-Wdeclaration-after-statement
	-Wframe-larger-than=2048
	-Wimplicit-fallthrough
	-Wundef
	-Wvla
	-Wno-unused-but-set-variable
	-Wno-unused-const-variable
	-Wno-unused-value
	-Werror)

if(ENABLE_TEST_COVERAGE)
	include(Coverage)
	target_coverage(mac)
endif()
include(Sanitize)
target_sanitize(mac)

if(ENABLE_MAC_UTEST)
	add_subdirectory(utest)
endif()
