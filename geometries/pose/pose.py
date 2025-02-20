import numpy as np
from geometries.pose.orientation import Orientation
from geometries.pose.position import Position
from scipy.spatial.transform import Rotation

# from px4_controller.geometries.geometries.pose.orientation import Orientation
# from px4_controller.geometries.geometries.pose.position import Position

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
    
    
    def flu_to_enu(self):
      """
      Convert from FLU (x-forward, y-left, z-up) to ENU (x-east, y-north, z-up)
      Rotate about the z-axis -90 degrees
      """
      R = Rotation.from_euler('z', -90, degrees=True)
      return self.rotate_any(R)

    def flu_from_enu(self):
      """
      Convert from ENU (x-east, y-north, z-up) to FLU (x-forward, y-left, z-up).
      Rotate about the z-axis +90 degrees
      """
      R = Rotation.from_euler('z', 90, degrees=True)
      return self.rotate_any(R)

    def flu_to_ned(self):
      """
      Convert from FLU (x-forward, y-left, z-up) to NED (x-north, y-east, z-down).
      Rotate about the x-axis 180 degrees
      The mapping is: 
          x_ned = x_flu,
          y_ned = -y_flu,
          z_ned = -z_flu.
      """
      R = Rotation.from_euler('x', 180, degrees=True)
      return self.rotate_any(R)

    def flu_from_ned(self):
      """
      Convert from NED (x-north, y-east, z-down) to FLU (x-forward, y-left, z-up).
      Same as flu_to_ned -- Rotate about the x-axis 180 degrees
      """
      return self.flu_to_ned()
        
    
    def rotate_any(self, R):
      """
      Rotates the current Pose by any rotation
      Returns a new Pose with its position and orientation rotated by the specified rotation
      """
      
      pos = np.array([self.position.x, self.position.y, self.position.z])
      new_pos = R.apply(pos)
      # Transform orientation (quaternion multiplication: R * original)
      orig_rot = Rotation.from_quat([self.orientation.qx,
                                      self.orientation.qy,
                                      self.orientation.qz,
                                      self.orientation.qw])
      new_rot = R * orig_rot
      new_quat = new_rot.as_quat() # in [qx, qy, qz, qw] order
      # Build and return a new Pose
      new_position_obj = Position(new_pos[0], new_pos[1], new_pos[2])
      new_orientation_obj = Orientation(new_quat[0], new_quat[1], new_quat[2], new_quat[3])
      return Pose(new_position_obj, new_orientation_obj)