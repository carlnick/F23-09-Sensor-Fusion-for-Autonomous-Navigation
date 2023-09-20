import numpy as np

def ellipsoid_fit(X, flag=0, equals='xy'):
    if len(X[0]) != 3:
        raise ValueError('Input data must have three columns!')
    else:
        x = X[:, 0]
        y = X[:, 1]
        z = X[:, 2]
    
    if len(x) < 9 and flag == 0:
        raise ValueError('Must have at least 9 points to fit a unique ellipsoid')
    if len(x) < 6 and flag == 1:
        raise ValueError('Must have at least 6 points to fit a unique oriented ellipsoid')
    if len(x) < 5 and flag == 2:
        raise ValueError('Must have at least 5 points to fit a unique oriented ellipsoid with two axes equal')
    if len(x) < 3 and flag == 3:
        raise ValueError('Must have at least 4 points to fit a unique sphere')

if flag == 0:
    # fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
    D = [x**2, 
         y**2, 
         z**2, 
         2 * x * y, 
         2 * x * z, 
         2 * y * z, 
         2 * x, 
         2 * y, 
         2 * z]  # ndatapoints x 9 ellipsoid parameters
elif flag == 1:
    # fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Gx + 2Hy + 2Iz = 1
    D = [x**2, 
         y**2, 
         z**2, 
         2 * x, 
         2 * y, 
         2 * z]  # ndatapoints x 6 ellipsoid parameters
elif flag == 2:
    # fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Gx + 2Hy + 2Iz = 1,
    # where A = B or B = C or A = C
    if equals == 'yz' or equals == 'zy':
        D = [y**2 + z**2, 
             x**2, 
             2 * x, 
             2 * y, 
             2 * z]
    elif equals == 'xz' or equals == 'zx':
        D = [x**2 + z**2, 
             y**2, 
             2 * x, 
             2 * y, 
             2 * z]
    else:
        D = [x**2 + y**2, 
             z**2, 
             2 * x, 
             2 * y, 
             2 * z]
else:
    # fit sphere in the form A(x^2 + y^2 + z^2) + 2Gx + 2Hy + 2Iz = 1
    D = [x**2 + y**2 + z**2, 
         2 * x, 
         2 * y, 
         2 * z]  # ndatapoints x 4 sphere parameters

v = np.linalg.solve(np.dot(D.T, D), np.dot(D.T, np.ones((x.shape[0], 1))))

if flag == 0:
    A = np.array([[v[0], v[3], v[4], v[6]],
                  [v[3], v[1], v[5], v[7]],
                  [v[4], v[5], v[2], v[8]],
                  [v[6], v[7], v[8], -1]])
    center = -np.linalg.solve(A[:3, :3], np.array([v[6], v[7], v[8]]))
    T = np.eye(4)
    T[3, :3] = center.T
    R = np.dot(np.dot(T, A), T.T)
    evals, evecs = np.linalg.eig(R[:3, :3] / -R[3, 3])
    radii = np.sqrt(1 / np.diag(evals))
else:
    if flag == 1:
        v = np.array([v[0], v[1], v[2], 0, 0, 0, v[3], v[4], v[5]])
    elif flag == 2:
        if equals == 'xz' or equals == 'zx':
            v = np.array([v[0], v[1], v[0], 0, 0, 0, v[2], v[3], v[4]])
        elif equals == 'yz' or equals == 'zy':
            v = np.array([v[1], v[0], v[0], 0, 0, 0, v[2], v[3], v[4]])
        else:
            v = np.array([v[0], v[0], v[1], 0, 0, 0, v[2], v[3], v[4]])
    else:
        v = np.array([v[0], v[0], v[0], 0, 0, 0, v[1], v[2], v[3]])
    center = -v[6:9] / v[0:3]
    gam = 1 + (v[6]**2 / v[0] + v[7]**2 / v[1] + v[8]**2 / v[2])
    radii = np.sqrt(gam / v[0:3])
    evecs = np.eye(3)
