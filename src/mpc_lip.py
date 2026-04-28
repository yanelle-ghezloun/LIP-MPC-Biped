import numpy as np
import casadi as ca

class MPC_LIP:
    """
    Model Predictive Controller for LIP-based bipedal locomotion.
    
    Optimizes the ZMP trajectory to track a desired COM reference
    while keeping the ZMP within the support polygon at all times.
    
    The ZMP constraint ensures dynamic stability — if ZMP leaves
    the support polygon, the robot falls.
    
    Reference:
        Wieber, P.B. (2006). "Trajectory Free Linear Model Predictive Control
        for Stable Walking in the Presence of Strong Perturbations"
        IEEE-RAS Humanoids 2006.
    """

    def __init__(self, A, B, horizon=40):
        self.A  = A
        self.B  = B
        self.N  = horizon
        self.nx = 2   
        self.nu = 1   

        # Cost weights
        self.q_x   = 1000.0    
        self.q_v   = 100.0    
        self.q_zmp = 0.01  

        self._build_solver()

    def _build_solver(self):
        opti = ca.Opti()

        # Decision variables
        self.X    = opti.variable(self.nx, self.N + 1)
        self.U    = opti.variable(self.nu, self.N)

        # Parameters
        self.X0       = opti.parameter(self.nx)        
        self.X_ref    = opti.parameter(self.nx, self.N + 1)  
        self.ZMP_min  = opti.parameter(self.N)         
        self.ZMP_max  = opti.parameter(self.N)         
        A = self.A
        B = self.B

        # Cost
        cost = 0
        for k in range(self.N):
            x_err = self.X[:, k] - self.X_ref[:, k]
            cost += self.q_x   * x_err[0]**2
            cost += self.q_v   * x_err[1]**2
            cost += self.q_zmp * self.U[0, k]**2

        # Terminal cost
        x_err_f = self.X[:, self.N] - self.X_ref[:, self.N]
        cost += 100 * self.q_x * x_err_f[0]**2

        opti.minimize(cost)

        # Constraints
        opti.subject_to(self.X[:, 0] == self.X0)

        for k in range(self.N):
            # Dynamics
            opti.subject_to(
                self.X[:, k+1] == A @ self.X[:, k] + B @ self.U[:, k]
            )
            # ZMP stability constraint
            opti.subject_to(self.U[0, k] >= self.ZMP_min[k])
            opti.subject_to(self.U[0, k] <= self.ZMP_max[k])

        opts = {'ipopt.print_level': 0, 'print_time': 0}
        opti.solver('ipopt', opts)

        self.opti = opti

    def solve(self, state, x_ref, zmp_min, zmp_max):
        """
        Solve MPC for current state.
        state   : current [x, x_dot]
        x_ref   : reference COM trajectory (2 x N+1)
        zmp_min : ZMP lower bounds (N,)
        zmp_max : ZMP upper bounds (N,)
        Returns optimal ZMP command.
        """
        self.opti.set_value(self.X0, state)
        self.opti.set_value(self.X_ref, x_ref)
        self.opti.set_value(self.ZMP_min, zmp_min)
        self.opti.set_value(self.ZMP_max, zmp_max)

        try:
            sol = self.opti.solve()
            return float(sol.value(self.U[0, 0]))
        except Exception:
            return float((zmp_min[0] + zmp_max[0]) / 2)
