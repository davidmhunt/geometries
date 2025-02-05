import numpy as np
from geometries.pose.orientation import Orientation
from geometries.pose.position import Position

class Pose:
    """
    A class to represent a full pose
    Defined in FLU coordinate frame (x-forward,y-left, z-up)
    
    Attributes:
    position (Position): the [x,y,z] position of the object
    orientation (orientation): the [x,y,z,w] rotation of the object
    """
    def __init__(self, position=None, orientation=None):
        self.position = position if position is not None else Position()
        self.orientation = orientation if orientation is not None else Orientation()
    
    def __repr__(self):
        return f"Pose(position={self.position}, orientation={self.orientation})"
    
    def to_dict(self):
        return {"position": {"x": self.position.x, "y": self.position.y, "z": self.position.z},
                "orientation": {"qx": self.orientation.qx, "qy": self.orientation.qy, "qz": self.orientation.qz, "qw": self.orientation.qw}}