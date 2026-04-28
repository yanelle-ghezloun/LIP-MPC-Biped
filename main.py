import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from src.lip_model import LIPModel
from src.mpc_lip import MPC_LIP
from src.gait_scheduler import GaitScheduler

# Parameters
h  = 0.8    
dt = 0.01   

# Init
lip       = LIPModel(h=h, dt=dt)
A, B      = lip.get_matrices()
mpc       = MPC_LIP(A, B, horizon=40)
scheduler = GaitScheduler(step_length=0.2, step_duration=0.8,
                          foot_size=0.1, dt=dt)

state = np.array([0.0, 0.0])  

# ── Storage
com_traj  = [state.copy()]
zmp_traj  = []
time_traj = [0.0]
supports  = []

print("Running LIP-MPC bipedal locomotion...")

# Main loop
t = 0.0
while not scheduler.is_done():
    # Get support polygon for horizon
    zmp_min, zmp_max = scheduler.get_support_sequence(horizon=40)

    # Get COM reference
    x_ref = scheduler.get_com_reference(horizon=40, current_x=state)

    # Solve MPC
    zmp_cmd = mpc.solve(state, x_ref, zmp_min, zmp_max)

    # Store support polygon
    s_min, s_max = scheduler.get_current_support()
    supports.append((t, s_min, s_max))

    # Simulate LIP
    state = lip.step(state, zmp_cmd)

    # Log
    com_traj.append(state.copy())
    zmp_traj.append(zmp_cmd)
    t += dt
    time_traj.append(t)

    # Update gait
    scheduler.update()

com_traj = np.array(com_traj)
zmp_traj = np.array(zmp_traj)
time_traj = np.array(time_traj)

print(f"Simulation done — {len(com_traj)} steps, {t:.2f}s")

# Plot 
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('LIP-MPC — Bipedal Locomotion (Exoskeleton Model)', fontsize=14)

axes[0, 0].plot(time_traj, com_traj[:, 0], 'b-', linewidth=2, label='COM position')
axes[0, 0].plot(time_traj[:-1], zmp_traj, 'r--', linewidth=1.5, label='ZMP command')

for (t_s, s_min, s_max) in supports[::10]:
    axes[0, 0].fill_between([t_s, t_s + 0.1], s_min, s_max,
                             alpha=0.15, color='green')

axes[0, 0].set_title('COM and ZMP Trajectories')
axes[0, 0].set_xlabel('Time (s)')
axes[0, 0].set_ylabel('Position (m)')
axes[0, 0].legend()
axes[0, 0].grid(True)

axes[0, 1].plot(time_traj, com_traj[:, 1], 'g-', linewidth=2)
axes[0, 1].set_title('COM Velocity')
axes[0, 1].set_xlabel('Time (s)')
axes[0, 1].set_ylabel('Velocity (m/s)')
axes[0, 1].grid(True)

axes[1, 0].plot(time_traj[:-1], zmp_traj, 'r-', linewidth=2, label='ZMP')
sup_arr = np.array(supports)
axes[1, 0].fill_between(sup_arr[:, 0],
                         sup_arr[:, 1], sup_arr[:, 2],
                         alpha=0.3, color='green', label='Support polygon')
axes[1, 0].set_title('ZMP vs Support Polygon')
axes[1, 0].set_xlabel('Time (s)')
axes[1, 0].set_ylabel('Position (m)')
axes[1, 0].legend()
axes[1, 0].grid(True)

ax = axes[1, 1]
footsteps = scheduler.get_footsteps()
for i, (fx, side) in enumerate(footsteps):
    color = 'steelblue' if side == 'left' else 'coral'
    rect = patches.Rectangle((fx - 0.05, -0.1), 0.1, 0.2,
                               linewidth=1, edgecolor='black',
                               facecolor=color, alpha=0.7)
    ax.add_patch(rect)
    ax.text(fx, 0.25, f'{i+1}', ha='center', fontsize=8)

ax.plot(com_traj[:, 0], np.zeros(len(com_traj)), 'b-',
        linewidth=2, label='COM path')
ax.set_xlim(-0.2, footsteps[-1][0] + 0.3)
ax.set_ylim(-0.4, 0.5)
ax.set_title('Footstep Sequence & COM Path')
ax.set_xlabel('Position (m)')
ax.legend()
ax.grid(True)

import matplotlib.patches as mpatches
left_patch  = mpatches.Patch(color='steelblue', label='Left foot')
right_patch = mpatches.Patch(color='coral', label='Right foot')
ax.legend(handles=[left_patch, right_patch,
          plt.Line2D([0], [0], color='blue', label='COM path')])

plt.tight_layout()
plt.savefig('results/lip_mpc_result.png', dpi=150)
plt.show()

print("Done: results/lip_mpc_result.png saved")

