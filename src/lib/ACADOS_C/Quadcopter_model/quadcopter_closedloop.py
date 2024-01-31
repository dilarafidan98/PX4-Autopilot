from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from quadcopter_model import quadcopter_ode_model
from utils import plot_drone
import numpy as np
import scipy.linalg
from casadi import vertcat
 
def setup(x0,x_goal, Umax, N_horizon, Ts):
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
 
    # set model
    model = quadcopter_ode_model()
    ocp.model = model
 
    nx =model.x.size()[0]
    nu = model.u.size()[0]
    ny = nu+nx
    ny_e = nx
 
 
    ocp.dims.N = N_horizon
 
    # set cost module
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    
 
    Q_mat = np.diag([100, 10, 100, 7*100000,50000,4000])
    R_mat = np.diag([0.001,0.01,0.001])
 
    #ocp.cost.W = np.block([[Q_mat, np.zeros((nx, nu))], [np.zeros((nu, nx)), R_mat]])
    #ocp.cost.W_e = Q_mat
 
    ocp.cost.W=np.diag(np.concatenate([np.diag(Q_mat), np.diag(R_mat)]))
    ocp.cost.W_e= Q_mat
    ocp.cost.Vx_e = np.eye(ny_e, nx)
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vu = np.zeros((ny,nu))

    #ocp.constraints.idxbx_e = np.array(range(nx))
    #ocp.constraints.lbx_e = x_goal
    #ocp.constraints.ubx_e = x_goal


 
 
    #ocp.model.cost_y_expr = vertcat(model.x, model.u)
    #ocp.model.cost_y_expr_e = model.x
    #ocp.model.cost_expr_ext_cost_e=model.x.T @ Q_mat @ model.x
    #ocp.model.cost_expr_ext_cost_e= (model.x.T @ Q_mat @ model.x) +(model.u.T @ R_mat @ model.u)


    #ocp.cost.yref  = np.zeros((ny,))
    #ocp.cost.yref_e = np.zeros((ny_e, ))

    ocp.cost.yref = np.concatenate([x_goal, np.zeros(nu)])  # Include goal state and zero control
    ocp.cost.yref_e = x_goal  # Terminal reference is the goal state
    
 
    
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.lbu = np.array([-Umax[0],-Umax[1],-Umax[2]])
    ocp.constraints.ubu = np.array([+Umax[0],+Umax[1],+Umax[2]])
 
    ocp.constraints.x0 = x0

   
 
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES' # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_newton_iter = 5
 
    
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
   
 
    ocp.solver_options.qp_solver_cond_N = N_horizon
 
    # set prediction horizon
    ocp.solver_options.tf = N_horizon*Ts
 
    solver_json = model.name + '.json'
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = solver_json)
 
    # create an integrator with the same settings as used in the OCP solver.
    acados_integrator = AcadosSimSolver(ocp, json_file = solver_json)
 
    return acados_ocp_solver, acados_integrator
 
 
def main():
 
    #x0 = np.array([np.deg2rad(270), np.deg2rad(9.42) , np.deg2rad(-40), 10, 1.20, 0.77])
    x0 = np.array([0, 0,0, 0,0, 0])
   
    #set constrains
    taux_max=20
    tauy_max=20
    tauz_max=10
   
    Tf = 0.01
    N_horizon = 25


    # terminal constraint
    x_goal = np.array([0.0929,0.1993, -0.869029,-0.1218 ,0.3470,-1.15])
    #x_goal = np.array([0.,0.005614494, -0.255522519, 0.015142142,-0.002383463,0.000181737])
   
 
    ocp_solver, integrator = setup(x0, x_goal,[taux_max,tauy_max,tauz_max] , N_horizon, Tf)
 
    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
 
    Nsim = 20
    simX = np.ndarray((Nsim+1, nx))
    simU = np.ndarray((Nsim, nu))
 
    simX[0,:] = x0
   
 

    t_preparation = np.zeros((Nsim))
    t_feedback = np.zeros((Nsim))

    # do some initial iterations to start with a good initial guess
    num_iter_initial = 5
    for _ in range(num_iter_initial):
        ocp_solver.solve_for_x0(x0_bar = x0)
 


 
    # closed loop
    for i in range(Nsim):
 
    
        # preparation phase
        ocp_solver.options_set('rti_phase', 1)
        status = ocp_solver.solve()
        print("Status: ",status)
        t_preparation[i] = ocp_solver.get_stats('time_tot')
 
        # set initial state
        ocp_solver.set(0, "lbx", simX[i, :])
        ocp_solver.set(0, "ubx", simX[i, :])
                 
 
            
        # feedback phase
        ocp_solver.options_set('rti_phase', 2)
        status = ocp_solver.solve()
        t_feedback[i] = ocp_solver.get_stats('time_tot')
 
        simU[i, :] = ocp_solver.get(0, "u")
 
 
        # simulate system
        simX[i+1, :] = integrator.simulate(x=simX[i, :], u=simU[i,:])
    print(simX)
 
    # evaluate timings

    # scale to milliseconds
    t_preparation *= 1000
    t_feedback *= 1000
    print(f'Computation time in preparation phase in ms: \
                min {np.min(t_preparation):.3f} median {np.median(t_preparation):.3f} max {np.max(t_preparation):.3f}')
    print(f'Computation time in feedback phase in ms:    \
                min {np.min(t_feedback):.3f} median {np.median(t_feedback):.3f} max {np.max(t_feedback):.3f}')
    
    # plot results
    plot_drone(np.linspace(0, (Tf/N_horizon)*Nsim, Nsim+1), [taux_max,tauy_max,tauz_max], simU, simX)
 
    ocp_solver = None
 
if __name__ == '__main__':
    main()

