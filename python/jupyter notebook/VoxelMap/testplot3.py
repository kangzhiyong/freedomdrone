import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

x = np.zeros(100)
y = np.zeros(100)
z = np.zeros(100)

z_inc = 4.0/99.0; 
theta_inc = (8.0 * 3.14)/99.0;

for i in range(100):
    theta = -4.0 * 3.14 + theta_inc*i
    z[i] = -2.0 + z_inc*i
    r = z[i]*z[i] + 1
    x[i] = (r * sin(theta))
    y[i] = (r * cos(theta))


plt.plot(x, y, z, projection='3d')
plt.xlabel("x label")
plt.ylabel("y label")
plt.set_zlabel("z label")
plt.legend()
plt.show()