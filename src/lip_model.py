import numpy as np

class LIPModel:
    """
    Linear Inverted Pendulum (LIP) Model.
    
    Models the center of mass (COM) dynamics of a bipedal system
    as a point mass at constant height h, supported by a massless rigid leg.
    
    Derived from Lagrangian mechanics under the constraint z = h = constant.
    
    Equation of motion (sagittal plane):
        x_ddot = omega^2 * (x - p)
    where:
        omega = sqrt(g/h) — natural frequency
        x     — COM position (m)
        p     — ZMP position (m)
    
    Reference:
        Kajita et al. (2001). "The 3D Linear Inverted Pendulum Mode"
        IEEE/RSJ IROS 2001.
    """

    def __init__(self, h=0.8, dt=0.01):

        self.h     = h
        self.dt    = dt
        self.g     = 9.81
        self.omega = np.sqrt(self.g / self.h)
        self.A, self.B = self._discretize()

    def _discretize(self):
        """
        Discretize continuous LIP dynamics using Euler method.
        
        Continuous:
            Ac = [[0,       1  ],
                  [omega^2, 0  ]]
            Bc = [[0       ],
                  [-omega^2]]
        
        Discrete (Euler):
            A = I + Ac*dt
            B = Bc*dt
        """
        w2 = self.omega ** 2
        dt = self.dt

        A = np.array([
            [1,      dt   ],
            [w2*dt,  1    ]
        ])

        B = np.array([
            [0        ],
            [-w2 * dt ]
        ])

        return A, B

    def step(self, state, zmp):
        """
        Simulate one timestep.
        state : [x, x_dot]
        zmp   : ZMP position (m)
        Returns next state.
        """
        return self.A @ state + self.B.flatten() * zmp

    def get_matrices(self):
        return self.A, self.B

    @property
    def natural_frequency(self):
        return self.omega
