import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

x = X_axis
y = Y_axis
z = Z_axis

# do ellipsoid fitting
e_center, e_radii, e_eigenvecs, e_algebraic = ellipsoid_fit(np.column_stack((x, y, z)))

# compensate distorted magnetometer data
# e_eigenvecs is an orthogonal matrix, so ' can be used instead of inv()
S = np.transpose(np.array([x - e_center[0], y - e_center[1], z - e_center[2]])) # translate and make array
scale = np.linalg.inv(np.array([[e_radii[0], 0, 0], [0, e_radii[1], 0], [0, 0, e_radii[2]]])) * min(e_radii) # scaling matrix
map = np.transpose(e_eigenvecs) # transformation matrix to map ellipsoid axes to coordinate system axes
invmap = e_eigenvecs # inverse of above
comp = np.dot(np.dot(invmap, scale), map)
S = np.dot(comp, S) # do compensation

# output info
print('#define CALIBRATION__MAGN_USE_EXTENDED true')
print('const float magn_ellipsoid_center[3] = [{:.6g}, {:.6g}, {:.6g}];'.format(*e_center))
print('const float magn_ellipsoid_transform[3][3] = {{[{:.6g}, {:.6g}, {:.6g}], [{:.6g}, {:.6g}, {:.6g}], [{:.6g}, {:.6g}, {:.6g}]}};'.format(*comp.flatten()))

# draw ellipsoid fit
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z, color='r') # original data
maxd = max(e_radii)
step = maxd / 50
xp, yp, zp = np.meshgrid(np.arange(-maxd, maxd + e_center[0], step), np.arange(-maxd, maxd + e_center[1], step), np.arange(-maxd, maxd + e_center[2], step))

Ellipsoid = e_algebraic(1) * xp * xp + e_algebraic(2) * yp * yp + e_algebraic(3) * zp * zp + \
    2 * e_algebraic(4) * xp * yp + 2 * e_algebraic(5) * xp * zp + 2 * e_algebraic(6) * yp * zp + \
    2 * e_algebraic(7) * xp + 2 * e_algebraic(8) * yp + 2 * e_algebraic(9) * zp
	
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

xp, yp, zp = np.meshgrid(xp, yp, zp)
p = ax.plot_surface(xp, yp, zp, Ellipsoid, cmap='Greens', edgecolor='none', alpha=0.5)

ax.view_init(-70, 40)
ax.axis('vis3d')
ax.axis('equal')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(x, y, z, c='r', marker='.')
ax.scatter(S[0,:], S[1,:], S[2,:], c='b', marker='.')

ax.view_init(-70, 40)
ax.set_box_aspect([1,1,1])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
