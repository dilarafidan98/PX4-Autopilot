#include "mpc_rate_solver.hpp"
#include <px4_platform_common/defines.h>
#include <iostream>
#include "../ACADOS_C/c_generated_code/acados_solver_Quadcopter_model_ode.h"
#define QUADCOPTER_MODEL_ODE_NX     6
#define QUADCOPTER_MODEL_ODE_NZ     0
#define QUADCOPTER_MODEL_ODE_NU     3
#define QUADCOPTER_MODEL_ODE_NP     0
#define QUADCOPTER_MODEL_ODE_NBX    0
#define QUADCOPTER_MODEL_ODE_NBX0   6
#define QUADCOPTER_MODEL_ODE_NBU    3
#define QUADCOPTER_MODEL_ODE_NSBX   0
#define QUADCOPTER_MODEL_ODE_NSBU   0
#define QUADCOPTER_MODEL_ODE_NSH    0
#define QUADCOPTER_MODEL_ODE_NSH0   0
#define QUADCOPTER_MODEL_ODE_NSG    0
#define QUADCOPTER_MODEL_ODE_NSPHI  0
#define QUADCOPTER_MODEL_ODE_NSHN   0
#define QUADCOPTER_MODEL_ODE_NSGN   0
#define QUADCOPTER_MODEL_ODE_NSPHIN 0
#define QUADCOPTER_MODEL_ODE_NSPHI0 0
#define QUADCOPTER_MODEL_ODE_NSBXN  0
#define QUADCOPTER_MODEL_ODE_NS     0
#define QUADCOPTER_MODEL_ODE_NS0    0
#define QUADCOPTER_MODEL_ODE_NSN    0
#define QUADCOPTER_MODEL_ODE_NG     0
#define QUADCOPTER_MODEL_ODE_NBXN   0
#define QUADCOPTER_MODEL_ODE_NGN    0
#define QUADCOPTER_MODEL_ODE_NY0    9
#define QUADCOPTER_MODEL_ODE_NY     9
#define QUADCOPTER_MODEL_ODE_NYN    6
#define QUADCOPTER_MODEL_ODE_N      25
#define QUADCOPTER_MODEL_ODE_NH     0
#define QUADCOPTER_MODEL_ODE_NHN    0
#define QUADCOPTER_MODEL_ODE_NH0    0
#define QUADCOPTER_MODEL_ODE_NPHI0  0
#define QUADCOPTER_MODEL_ODE_NPHI   0
#define QUADCOPTER_MODEL_ODE_NPHIN  0
#define QUADCOPTER_MODEL_ODE_NR     0

namespace acados_rate {
 acados_quadcopter::acados_quadcopter() {

 }

 acados_quadcopter::~acados_quadcopter() {
  if (acados_ocp_capsule == nullptr) {
    std::cout << "freeing deleted acados_quadcopter" << std::endl;
    return;
  }

  std::cout << "freeing acados_quadcopter" << std::endl;
  // free solver
  int status = Quadcopter_model_ode_acados_free(acados_ocp_capsule);
  if (status) {
    printf("quadcopter_acados_free() returned status %d. \n", status);
  }
  // free solver capsule
  status = Quadcopter_model_ode_acados_free_capsule(acados_ocp_capsule);
  if (status) {
    printf("quadcopter_acados_free_capsule() returned status %d. \n", status);
  }
  acados_ocp_capsule = nullptr;
}

bool acados_quadcopter::acadosinitialize(){
 acados_ocp_capsule=Quadcopter_model_ode_acados_create_capsule();

  std::cout << "capsule created" << std::endl;
 //int status = quadcopter_acados_create_with_discretization(acados_ocp_capsule);
 int status=Quadcopter_model_ode_acados_create(acados_ocp_capsule);
  if (status) {
    std::cerr << "quadcopter_acados_create_with_discretization() returned status:"
              << status << "\n";
    return false;
  }
   nlp_config = Quadcopter_model_ode_acados_get_nlp_config(acados_ocp_capsule);
   nlp_dims = Quadcopter_model_ode_acados_get_nlp_dims(acados_ocp_capsule);
   nlp_in = Quadcopter_model_ode_acados_get_nlp_in(acados_ocp_capsule);
   nlp_out = Quadcopter_model_ode_acados_get_nlp_out(acados_ocp_capsule);
   nlp_solver = Quadcopter_model_ode_acados_get_nlp_solver(acados_ocp_capsule);
   nlp_opts = Quadcopter_model_ode_acados_get_nlp_opts(acados_ocp_capsule);
   return true;

       std::cerr << "quadcopter_acados_create_with_discretization() returned status:"
              << status << "\n";

    // Set input constraints
    double lbu[QUADCOPTER_MODEL_ODE_NU];
    double ubu[QUADCOPTER_MODEL_ODE_NU];

    lbu[0] = -20;
    ubu[0] = 20;
    lbu[1] = -20;
    ubu[1] = 20;
    lbu[2] = -5;
    ubu[2] = 5;

    for (int i = 0; i < N; i++)
    {

        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
}


void acados_quadcopter::set_init_state(matrix::Vector3f initialx_angle, matrix::Vector3f initialx_rate) {
  // Print initialx_angle
        std::cout << "Initial X Angle: ["
                  << initialx_angle(0) << ", "
                  << initialx_angle(1) << ", "
                  << initialx_angle(2) << "]" << std::endl;

        // Print initialx_rate
        std::cout << "Initial X Rate: ["
                  << initialx_rate(0) << ", "
                  << initialx_rate(1) << ", "
                  << initialx_rate(2) << "]" << std::endl;
  int idxbx0[6];
  double lbx0[6];
  double ubx0[6];

  idxbx0[0] = 0;
  lbx0[0] = initialx_angle(0);
  ubx0[0] = initialx_angle(0);
  idxbx0[1] = 1;
  lbx0[1] = initialx_angle(1);
  ubx0[1] = initialx_angle(1);
  idxbx0[2] = 2;
  lbx0[2] = initialx_angle(2);
  ubx0[2] = initialx_angle(2);
  idxbx0[3] = 3;
  lbx0[3] = initialx_rate(0);
  ubx0[3] = initialx_rate(0);
  idxbx0[4] = 4;
  lbx0[4] = initialx_rate(1);
  ubx0[4] = initialx_rate(1);
  idxbx0[5] = 5;
  lbx0[5] = initialx_rate(2);
  ubx0[5] = initialx_rate(2);

  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);

  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

}

/*void acados_quadcopter::set_init_solution(matrix::Vector3f initialx_angle, matrix::Vector3f initialx_rate){
  double x_init[QUADCOPTER_NX];
  double u0[QUADCOPTER_NU];

  // Print initialx_angle
        std::cout << "Initial X Angle: ["
                  << initialx_angle(0) << ", "
                  << initialx_angle(1) << ", "
                  << initialx_angle(2) << "]" << std::endl;

        // Print initialx_rate
        std::cout << "Initial X Rate: ["
                  << initialx_rate(0) << ", "
                  << initialx_rate(1) << ", "
                  << initialx_rate(2) << "]" << std::endl;


   x_init[0] = initialx_angle(0);
   x_init[1] = initialx_angle(1);
   x_init[2] = initialx_angle(2);
   x_init[3] = initialx_rate(0);
   x_init[4] = initialx_rate(1);
   x_init[5] = initialx_rate(2);

   // initial value for control input
   u0[0] = 0;
   u0[1] = 0;
   u0[2] = 0;


  // initialize solution
  for (int i = 0; i < N; i++) {
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
  }
  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
}*/


void acados_quadcopter::compute_control() {
  // solve ocp in loop
  int rti_phase = 0;
  ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
  int status = Quadcopter_model_ode_acados_solve(acados_ocp_capsule);
  printf("problem %d\n", status);

}

double acados_quadcopter::get_elapsed_time() {
  double elapsed_time;
  ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
  return elapsed_time;
}

void acados_quadcopter::print_stats() {
  int sqp_iter, stat_m, stat_n;
  ocp_nlp_get(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_solver, "sqp_iter", &sqp_iter);
  ocp_nlp_get(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_solver, "stat_n", &stat_n);
  ocp_nlp_get(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_solver, "stat_m", &stat_m);

  double stat[1200];
  ocp_nlp_get(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_solver, "statistics", stat);

  int qp_iter = (int)stat[2 + 1 * 3];
  double elapsed_time = get_elapsed_time();
  std::cout << "qp iterations: " << qp_iter << ", time elapsed: " << elapsed_time * 1000 << " ms." << std::endl;
}

matrix::Vector3f acados_quadcopter::get_first_control_action() {
  matrix::Vector3f utraj;
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &utraj(0));

  return utraj;
}




void acados_quadcopter::set_all_init_solutions(double xtraj[], double utraj[]) {
  // initialize solution


  for (int i = 0; i < N; i++) {
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", &xtraj[i * QUADCOPTER_MODEL_ODE_NX]);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", &utraj[i * QUADCOPTER_MODEL_ODE_NX]);
  }
  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", &xtraj[N * QUADCOPTER_MODEL_ODE_NX]);
}

void acados_quadcopter::get_all_init_solutions(double xtraj[], double utraj[]) {

  for (int ii = 0; ii <= nlp_dims-> N-1; ii++)
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii + 1, "x", &xtraj[ii * QUADCOPTER_MODEL_ODE_NX]);
  for (int ii = 0; ii < nlp_dims->N-1; ii++)
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii + 1, "u", &utraj[ii * QUADCOPTER_MODEL_ODE_NU]);
  //ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, nlp_dims->N, "x", &xtraj[(nlp_dims->N) * NX]);
  //ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, nlp_dims->N - 1, "u", &utraj[(nlp_dims->N - 1) * NU]);
}

}


