import numpy as np
from scipy.spatial.transform import Rotation

class Orientation:
    """
    A class to represent an orientation in 3D space using a quaternion.
    Defined in FLU coordinate frame (x-forward,y-left, z-up)
    
    Attributes:
    qx (float): Quaternion x-component.
    qy (float): Quaternion y-component.
    qz (float): Quaternion z-component.
    qw (float): Quaternion w-component.
    """
    
    def __init__(self, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        Initializes an Orientation object with quaternion components.
        
        Parameters:
        qx (float): Initial x-component of quaternion. Default is 0.0.
        qy (float): Initial y-component of quaternion. Default is 0.0.
        qz (float): Initial z-component of quaternion. Default is 0.0.
        qw (float): Initial w-component of quaternion. Default is 1.0.
        """
        self._orientation = np.array([qx, qy, qz, qw], dtype=np.float64)
    
    @property
    def qx(self):
        """Gets the x-component of the quaternion."""
        return self._orientation[0]
    
    @qx.setter
    def qx(self, value):
        """Sets the x-component of the quaternion."""
        self._orientation[0] = value
    
    @property
    def qy(self):
        """Gets the y-component of the quaternion."""
        return self._orientation[1]
    
    @qy.setter
    def qy(self, value):
        """Sets the y-component of the quaternion."""
        self._orientation[1] = value
    
    @property
    def qz(self):
        """Gets the z-component of the quaternion."""
        return self._orientation[2]
    
    @qz.setter
    def qz(self, value):
        """Sets the z-component of the quaternion."""
        self._orientation[2] = value
    
    @property
    def qw(self):
        """Gets the w-component of the quaternion."""
        return self._orientation[3]
    
    @qw.setter
    def qw(self, value):
        """Sets the w-component of the quaternion."""
        self._orientation[3] = value
    
    def __repr__(self):
        """Returns a string representation of the Orientation object."""
        return f"Orientation(qx={self.qx}, qy={self.qy}, qz={self.qz}, qw={self.qw})"

    @classmethod
    def from_euler(cls, roll:float=0.0, pitch:float=0.0, yaw:float=0.0, degrees=False):
        """
        Creates an Orientation object from Euler angles (roll, pitch, yaw).
        
        Parameters:
        roll (float): Rotation around the x-axis in radians.Default is 0.0.
        pitch (float): Rotation around the y-axis in radians. Default is 0.0.
        yaw (float): Rotation around the z-axis in radians. Default is 0.0.
        degrees(bool): on True, given angles are in degrees. Else in radians.
            Defaults to False
        
        Returns:
        Orientation: An instance of Orientation initialized from the Euler angles.
        """
        quaternion = Rotation.from_euler('xyz', [roll, pitch, yaw],degrees=degrees).as_quat()
        return cls(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

    def to_euler(self,degrees=False):
        """
        Converts the stored quaternion to Euler angles (roll, pitch, yaw).
        
        Parameters:
        degrees(bool): on True, returned values are in degrees. Else in radians.
            Defaults to False

        Returns:
        np.ndarray: A tuple (roll, pitch, yaw) representing the Euler angles in radians.
        """
        rotation = Rotation.from_quat(self._orientation)

        return rotation.as_euler('xyz', degrees=degrees)
        
