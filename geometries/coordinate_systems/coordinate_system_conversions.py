import numpy as np

####################################################################
#Spherical and Cartesian
#################################################################### 

def spherical_to_cartesian(points:np.ndarray)->np.ndarray:
    """
    Convert an Nx3 array of spherical coordinates to Cartesian coordinates.

    Parameters:
        points (ndarray): Nx3 array where each row is (r, theta - from x, phi - from z) in radians.

    Returns:
        ndarray: Nx3 array where each row is (x, y, z).
    """
    if (points.ndim == 2 and points.shape[1] == 3):
       
        r = points[:, 0]
        theta = points[:, 1]
        phi = points[:, 2]

        x = r * np.sin(phi) * np.cos(theta)
        y = r * np.sin(phi) * np.sin(theta)
        z = r * np.cos(phi)

        return np.column_stack((x, y, z))
    else:
        raise ValueError("spherical_to_cartesian: input points must be an Nx3 array of points.")

def cartesian_to_spherical(points):
    """
    Convert an Nx3 array of Cartesian coordinates to spherical coordinates.

    Parameters:
        points (ndarray): Nx3 array where each row is (x, y, z).

    Returns:
        ndarray: Nx3 array where each row is (r, theta - from x, phi - from z).
    """
    if (points.ndim == 2 and points.shape[1] == 3):
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        r = np.sqrt(x**2 + y**2 + z**2)
        theta = np.arctan2(y, x)
        phi = np.arccos(np.clip(z / r, -1, 1))  # Clip to avoid numerical issues

        return np.column_stack((r, theta, phi))
    
    else:
        raise ValueError("cartesian_to_spherical: input points must be an Nx3 array of points.")

####################################################################
#Cartesian and Cylindrical
#################################################################### 

def cylindrical_to_cartesian(points:np.ndarray)->np.ndarray:
    """
    Convert an Nx3 array of spherical coordinates to Cartesian coordinates.

    Parameters:
        points (ndarray): Nx3 array where each row is (r, theta from x, z) in radians.

    Returns:
        ndarray: Nx3 array where each row is (x, y, z).
    """
    if (points.ndim == 2 and points.shape[1] == 3):
       
        r = points[:, 0]
        theta = points[:, 1]
        z = points[:, 2]

        x = r * np.cos(theta)
        y = r * np.sin(theta)

        return np.column_stack((x, y, z))
    else:
        raise ValueError("spherical_to_cartesian: input points must be an Nx3 array of points.")

def cartesian_to_cylindrical(points):
    """
    Convert an Nx3 array of Cartesian coordinates to spherical coordinates.

    Parameters:
        points (ndarray): Nx3 array where each row is (x, y, z).

    Returns:
        ndarray: cylindrical Nx3 array where each row is (r, theta from x, z) in radians.
    """
    if (points.ndim == 2 and points.shape[1] == 3):
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)

        return np.column_stack((r, theta, z))
    
    else:
        raise ValueError("cartesian_to_spherical: input points must be an Nx3 array of points.")

