#include "c_generated_code/acados_solver_quadcopter.h"

#include "acados/utils/types.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#include <matrix/matrix/math.hpp>

#include <mathlib/mathlib.h>
#include <uORB/topics/rate_ctrl_status.h>



class acados_quadcopter{
	private:
	   quadcopter_solver_capsule *acados_ocp_capsule;
	   ocp_nlp_config *nlp_config;
	   ocp_nlp_dims *nlp_dims;
	   ocp_nlp_in *nlp_in;
	   ocp_nlp_out *nlp_out;
	   ocp_nlp_solver *nlp_solver;
	   void *nlp_opts;
	   //ocp_nlp_plan_t *nlp_solver_plan;
	   //add other values
	public:
	    int N;//number of shooting intervals
	    double dt;//time interval
	    acados_quadcopter();
	    acados_quadcopter(int _N,double *_time_steps);
	    acados_quadcopter(acados_quadcopter &&other);
	    ~acados_quadcopter();
	    acados_quadcopter &operator=(acados_quadcopter &&other);

	    void set_init_state();
	    void set_init_solution(matrix::Vector3f &initialx_angle,matrix::Vector3f &initialx_rate,matrix::Vector3f &initialu);
	    void set_all_init_solutions(double xtraj[],double utraj[]);
	    void get_all_init_solutions(double xtraj[],double utraj[]);
	    //void set_ref
	    void compute_control();
	    double get_elapsed_time();
	    matrix::Vector3f get_first_control_action();
	    void print_stats();
	    //void setNewCosts


};









