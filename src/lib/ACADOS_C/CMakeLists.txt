# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB_RECURSE GENERATED_C_CODE RELATIVE ${CMAKE_SOURCE_DIR} "c_generated_code/")

px4_add_library(ACADOS_C

	c_generated_code/acados_solver_quadcopter.h
	${GENERATED_C_CODE}
)

target_compile_options(ACADOS_C PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(ACADOS_C PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ACADOS_C PRIVATE mathlib)
target_link_libraries(ACADOS_C PUBLIC ${ACADOS_LIB})
#target_link_libraries(ACADOS_C PUBLIC ${C_GENERATED_LIB})



