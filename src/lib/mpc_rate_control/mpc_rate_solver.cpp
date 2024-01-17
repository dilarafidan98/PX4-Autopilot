#include "mpc_rate_solver.hpp"
#include <px4_platform_common/defines.h>
#include <iostream>
#include "c_generated_code/acados_solver_quadcopter.h"
#define NX     QUADCOPTER_NX
#define NZ     QUADCOPTER_NZ
#define NU     QUADCOPTER_NU
#define NP     QUADCOPTER_NP
#define NY0    QUADCOPTER_NY0
#define NY     QUADCOPTER_NY
#define NYN    QUADCOPTER_NYN

#define NBX    QUADCOPTER_NBX
#define NBX0   QUADCOPTER_NBX0
#define NBU    QUADCOPTER_NBU
#define NG     QUADCOPTER_NG
#define NBXN   QUADCOPTER_NBXN
#define NGN    QUADCOPTER_NGN

#define NH     QUADCOPTER_NH
#define NHN    QUADCOPTER_NHN
#define NH0    QUADCOPTER_NH0
#define NPHI   QUADCOPTER_NPHI
#define NPHIN  QUADCOPTER_NPHIN
#define NPHI0  QUADCOPTER_NPHI0
#define NR     QUADCOPTER_NR

#define NS     QUADCOPTER_NS
#define NS0    QUADCOPTER_NS0
#define NSN    QUADCOPTER_NSN

#define NSBX   QUADCOPTER_NSBX
#define NSBU   QUADCOPTER_NSBU
#define NSH0   QUADCOPTER_NSH0
#define NSH    QUADCOPTER_NSH
#define NSHN   QUADCOPTER_NSHN
#define NSG    QUADCOPTER_NSG
#define NSPHI0 QUADCOPTER_NSPHI0
#define NSPHI  QUADCOPTER_NSPHI
#define NSPHIN QUADCOPTER_NSPHIN
#define NSGN   QUADCOPTER_NSGN
#define NSBXN  QUADCOPTER_NSBXN


acados_quadcopter::acados_quadcopter():acados_quadcopter(QUADCOPTER_N, NULL){}

acados_quadcopter::acados_quadcopter(int _N,double *new_time_steps): N(_N){
  std::cout << "creating acados_quadcopter" << std::endl;
  acados_ocp_capsule = quadcopter_acados_create_capsule();
  // there is an opportunity to change the number of shooting intervals in C without new code generation
  int status = quadcopter_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

  if (status) {
    printf("quadcopter_acados_create_capsule() returned status %d. Exiting.\n", status);
    exit(1);
  }

  //todo for dilara: search for dt

  if (new_time_steps == NULL) {
    dt = 2.0 / 30.0;
  } else {
    dt = new_time_steps[0];
  }

  nlp_config = quadcopter_acados_get_nlp_config(acados_ocp_capsule);
  nlp_dims = quadcopter_acados_get_nlp_dims(acados_ocp_capsule);
  nlp_in = quadcopter_acados_get_nlp_in(acados_ocp_capsule);
  nlp_out = quadcopter_acados_get_nlp_out(acados_ocp_capsule);
  nlp_solver = quadcopter_acados_get_nlp_solver(acados_ocp_capsule);
  nlp_opts = quadcopter_acados_get_nlp_opts(acados_ocp_capsule);

}

acados_quadcopter::~acados_quadcopter() {
  if (acados_ocp_capsule == nullptr) {
    std::cout << "freeing deleted acados_quadcopter" << std::endl;
    return;
  }

  std::cout << "freeing acados_quadcopter" << std::endl;
  // free solver
  int status = quadcopter_acados_free(acados_ocp_capsule);
  if (status) {
    printf("quadcopter_acados_free() returned status %d. \n", status);
  }
  // free solver capsule
  status = quadcopter_acados_free_capsule(acados_ocp_capsule);
  if (status) {
    printf("quadcopter_acados_free_capsule() returned status %d. \n", status);
  }
  acados_ocp_capsule = nullptr;
}
acados_quadcopter::acados_quadcopter(acados_quadcopter &&other) {
  std::cout << "creating acados_quadcopter by move constructor" << std::endl;

  if (this != &other) {
    acados_ocp_capsule = other.acados_ocp_capsule;
    nlp_config = other.nlp_config;
    nlp_dims = other.nlp_dims;
    nlp_in = other.nlp_in;
    nlp_out = other.nlp_out;
    nlp_solver = other.nlp_solver;
    nlp_opts = other.nlp_opts;

    N = other.N;
    dt = other.dt;

    other.acados_ocp_capsule = nullptr;
  }
}
acados_quadcopter &acados_quadcopter::operator=(acados_quadcopter &&other) {
  std::cout << "creating acados_quadcopter by move operator" << std::endl;

  if (this != &other) {
    this->~acados_quadcopter();

    acados_ocp_capsule = other.acados_ocp_capsule;
    nlp_config = other.nlp_config;
    nlp_dims = other.nlp_dims;
    nlp_in = other.nlp_in;
    nlp_out = other.nlp_out;
    nlp_solver = other.nlp_solver;
    nlp_opts = other.nlp_opts;

    N = other.N;
    dt = other.dt;

    other.acados_ocp_capsule = nullptr;
  }

  return *this;
}
void acados_quadcopter::set_init_state() {
  // initial condition
  double lbx0[NBX0];
  double ubx0[NBX0];

    lbx0[0] = 3.490658503988659;
    ubx0[0] = 3.490658503988659;
    lbx0[1] = 0;
    ubx0[1] = 0;
    lbx0[2] = 0;
    ubx0[2] = 0;
    lbx0[3] = 4.71238898038469;
    ubx0[3] = 4.71238898038469;
    lbx0[4] = 0;
    ubx0[4] = 0;
    lbx0[5] = 0;
    ubx0[5] = 0;


  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

  // ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, 0, "x", lbx0);
}
void acados_quadcopter::set_init_solution(matrix::Vector3f &initialx_angle, matrix::Vector3f &initialx_rate, matrix::Vector3f &initialu){
  double x_init[QUADCOPTER_NX];
  double u0[QUADCOPTER_NU];

   x_init[0] = initialx_angle(0);
   x_init[1] = initialx_angle(1);
   x_init[2] = initialx_angle(2);
   x_init[3] = initialx_rate(0);
   x_init[4] = initialx_rate(1);
   x_init[5] = initialx_rate(2);

   // initial value for control input
   u0[0] = initialu(0);
   u0[1] = initialu(1);
   u0[2] = initialu(2);


  // initialize solution
  for (int i = 0; i < N; i++) {
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
  }
  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
}

void acados_quadcopter::compute_control() {
  // solve ocp in loop
  int rti_phase = 0;
  ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
  int status = quadcopter_acados_solve(acados_ocp_capsule);

  if (status != ACADOS_SUCCESS) {
    printf("quadcopter_acados_solve() failed with status %d.\n", status);
  }
  // quadcopter_acados_print_stats(acados_ocp_capsule);
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
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", &xtraj[i * NX]);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", &utraj[i * NX]);
  }
  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", &xtraj[N * NX]);
}

void acados_quadcopter::get_all_init_solutions(double xtraj[], double utraj[]) {

  for (int ii = 0; ii <= nlp_dims-> N-1; ii++)
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii + 1, "x", &xtraj[ii * NX]);
  for (int ii = 0; ii < nlp_dims->N-1; ii++)
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii + 1, "u", &utraj[ii * NU]);
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, nlp_dims->N, "x", &xtraj[(nlp_dims->N) * NX]);
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, nlp_dims->N - 1, "u", &utraj[(nlp_dims->N - 1) * NU]);
}

