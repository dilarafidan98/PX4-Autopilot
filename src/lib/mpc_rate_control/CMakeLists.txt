
# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#file(GLOB_RECURSE GENERATED_C_CODE RELATIVE ${CMAKE_SOURCE_DIR} "c_generated_code")
px4_add_library(MPCRateControl
	mpc_rate_solver.cpp
	mpc_rate_solver.hpp
	#${GENERATED_C_CODE} # Generated c code
)

target_compile_options(MPCRateControl PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(MPCRateControl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(MPCRateControl PRIVATE mathlib)
target_link_libraries(MPCRateControl PUBLIC ${ACADOS_LIB})
target_link_libraries(MPCRateControl PUBLIC ACADOS_C)
target_link_libraries(MPCRateControl PUBLIC "/home/dilara/PX4-Autopilot/src/lib/ACADOS_C/c_generated_code/libacados_solver_quadcopter.so")

