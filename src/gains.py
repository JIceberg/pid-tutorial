import matplotlib.pyplot as plt
import numpy as np
from controller import PIDController
from quadrotor import ydot 

x = np.linspace(0, 45, 300)

fig, ax = plt.subplots()

# 0.65,0.11,0.1
controller = PIDController(0.65,0.11,0.1,5)
controller.setGoal(10)

y = [0, 0]

for t in x:
    y = ydot(y, t, controller)

ax.plot(x, controller.u_p, label='P', linewidth=3, color='red')
ax.plot(x, controller.u_i, label='I', linewidth=3, color='blue')
ax.plot(x, controller.u_d, label='D', linewidth=3, color='green')
ax.set_xlabel('time')
ax.set_ylabel('effort')
ax.set_title('Gain Effort Over Time')
ax.legend()

plt.show()
