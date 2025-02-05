import numpy as np

class Position:
    """
    A class to represent a 3D position in space.
    Defined in FLU coordinate frame (x-forward,y-left, z-up)
    
    Attributes:
    x (float): Position along the x-axis.
    y (float): Position along the y-axis.
    z (float): Position along the z-axis.
    """
    
    def __init__(self, x=0.0, y=0.0, z=0.0):
        """
        Initializes a Position object with x, y, and z coordinates.
        
        Parameters:
        x (float): Initial x-coordinate. Default is 0.0.
        y (float): Initial y-coordinate. Default is 0.0.
        z (float): Initial z-coordinate. Default is 0.0.
        """
        self._position = np.array([x, y, z], dtype=np.float64)
    
    @property
    def x(self):
        """Gets the x-coordinate."""
        return self._position[0]
    
    @x.setter
    def x(self, value):
        """Sets the x-coordinate."""
        self._position[0] = value
    
    @property
    def y(self):
        """Gets the y-coordinate."""
        return self._position[1]
    
    @y.setter
    def y(self, value):
        """Sets the y-coordinate."""
        self._position[1] = value
    
    @property
    def z(self):
        """Gets the z-coordinate."""
        return self._position[2]
    
    @z.setter
    def z(self, value):
        """Sets the z-coordinate."""
        self._position[2] = value
    
    def __repr__(self):
        """Returns a string representation of the Position object."""
        return f"Position(x={self.x}, y={self.y}, z={self.z})"
