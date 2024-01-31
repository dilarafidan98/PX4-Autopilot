#!/home/dilara/Desktop/Superbuild/build/acados-prefix/src/acados/.venv/bin/python
 
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#
from acados_template import AcadosModel
from casadi import SX, vertcat    
import numpy as np
import scipy.linalg
 
def quadcopter_ode_model() -> AcadosModel:
 
    
    model_name='Quadcopter_model_ode'
 
    
 
    # constants
    #Jxz = 0
    #Jz = 0.013
    #Jx = 0.0075
    #Jy = 0.0075
    #si = Jx * Jy - Jxz * Jxz
    #m = 2
    #g = 9.81
    #c3 = Jz / si
    #c4 = Jxz / si
    #c7 = 1 / Jy
    #c9 = Jx / si
 
   # State variables (example: roll, pitch, yaw and their rates)
    roll = SX.sym('roll')
    pitch = SX.sym('pitch')
    yaw = SX.sym('yaw')
    roll_rate = SX.sym('roll_rate')
    pitch_rate = SX.sym('pitch_rate')
    yaw_rate = SX.sym('yaw_rate')
    
 
    # Control inputs (example: torques)
    tau_x = SX.sym('tau_x')
    tau_y = SX.sym('tau_y')
    tau_z = SX.sym('tau_z')
 
    #Symbolic Variables
    x = vertcat(roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate)
    u = vertcat(tau_x, tau_y, tau_z)
    #xdot=vertcat(roll_dot,pitch_dot,yaw_dot,roll_rate_dot,pitch_rate_dot,yaw_rate_dot)
    #x = SX.sym('x', 6)
    #u = SX.sym('u', 3)
 

    #A=np.array([[1,0,0,0.01,0,0],[0,1,0,0,0.01,0],[0,0,1,0,0,0.01],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
    A=np.array([[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]])
    print("A:",A)
 

    #B = np.array([[0,0,0],[0,0,0],[0,0,0],[2.31,0,0],[0,1.33,0],[0,0,1.33]])
    B = np.array([[0,0,0],[0,0,0],[0,0,0],[231,0,0],[0,133,0],[0,0,133]])
 
 
    # Explicit dynamics
    #f_expl = vertcat(roll_rate,pitch_rate,yaw_rate,tau_x @ c3 + tau_z @ c4, c7 @ tau_y ,tau_x @ c4 + tau_z @ c9)
    #f_impl= xdot - f_expl
 
    f_xu = A @ x + B @ u
 
 
   
    # Acados model setup
    model = AcadosModel()
    model.x = x
    model.u = u
    model.f_expl_expr=f_xu
    #model.f_impl_expr=f_xu-model.x
    #model.f_expl_expr = f_expl
    #model.f_impl_expr=f_impl
    #model.xdot=xdot
    model.name = model_name
 
    
    
 
 
 
    return model
 
 