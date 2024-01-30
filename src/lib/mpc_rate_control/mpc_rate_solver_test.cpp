#include <gtest/gtest.h>
#include <lib/mpc_rate_control/mpc_rate_solver.hpp>

using namespace matrix;


TEST(MPCRateControlTest, AllZeroCase)
{

	acados_rate::acados_quadcopter mpc_rate;

	mpc_rate.acadosinitialize();

	//mpc_rate.set_init_state(matrix::Vector3f(4.7,0.164,-0.698),matrix::Vector3f(1,1,0.77));
	mpc_rate.set_init_state(matrix::Vector3f(),matrix::Vector3f());
	mpc_rate.compute_control();
	Vector3f TTTT =mpc_rate.get_first_control_action();

	EXPECT_EQ(TTTT,matrix::Vector3f());




}
