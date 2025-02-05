import numpy as np
from scipy.spatial.transform import Rotation
from geometries.pose.pose import Pose

class Transformation:
    """
    A class to represent a coordinate transformation from a parent frame to a child frame
    using a homogeneous transformation matrix.
    
    Attributes:
    translation (np.array): Translation vector (x, y, z) from parent to child frame.
    rotation (np.array): Quaternion rotation (qx, qy, qz, qw) from parent to child frame.
    """
    
    def __init__(self, translation=None, rotation=None):
        """
        Initializes the Transformation object with a translation vector and a quaternion rotation.
        
        Parameters:
        translation (np.array): Translation vector (x, y, z). Default is [0.0, 0.0, 0.0].
        rotation (np.array): Quaternion (qx, qy, qz, qw). Default is [0.0, 0.0, 0.0, 1.0] (identity quaternion).
        """
        self.translation = np.array(translation if translation is not None else [0.0, 0.0, 0.0])
        self.rotation = np.array(rotation if rotation is not None else [0.0, 0.0, 0.0, 1.0])

        # Compute the transformation matrix when the object is initialized
        self.transformation_matrix = self.compute_transformation_matrix()

    def compute_transformation_matrix(self):
        """
        Computes the 4x4 homogeneous transformation matrix from the quaternion and translation.
        
        Returns:
        np.array: 4x4 homogeneous transformation matrix.
        """
        # Convert quaternion to rotation matrix
        rot_matrix = Rotation.from_quat(self.rotation).as_matrix()

        # Create the 4x4 homogeneous transformation matrix
        transformation_matrix = np.eye(4)  # Start with an identity matrix
        transformation_matrix[:3, :3] = rot_matrix  # Rotation part
        transformation_matrix[:3, 3] = self.translation  # Translation part
        
        return transformation_matrix

    def apply_transformation(self, points):
        """
        Applies the transformation to a 3D point or an Nx3 array of points using the homogeneous transformation matrix.
        
        Parameters:
        points (np.array): A 3D point (3,) or an Nx3 array of points in the parent frame.
        
        Returns:
        np.array: Transformed 3D point(s) in the child frame.
        """
        # Check if points is a single point (3,) or an array of points (Nx3)
        if points.ndim == 1 and points.shape == (3,):
            # Single point case: Convert the point to homogeneous coordinates (add a 1)
            points_homogeneous = np.append(points, 1)
            # Apply the transformation matrix
            transformed_points_homogeneous = self.transformation_matrix.dot(points_homogeneous)
            # Return the transformed point (the first three coordinates)
            return transformed_points_homogeneous[:3]
        
        elif points.ndim == 2 and points.shape[1] == 3:
            # Multiple points case (Nx3): Convert each point to homogeneous coordinates
            points_homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])  # Add a column of 1's
            # Apply the transformation matrix to each point
            transformed_points_homogeneous = points_homogeneous.dot(self.transformation_matrix.T)
            # Return the transformed points (the first three columns)
            return transformed_points_homogeneous[:, :3]
        
        else:
            raise ValueError("Input points must be a 3D point (3,) or an Nx3 array of points.")

    @classmethod
    def from_global_poses(cls, parent_pose:Pose, child_pose:Pose):
        """
        Initializes a Transformation object from the global poses of the parent and child frames.
        The transform is initialized to go from parent to child
        
        Parameters:
        parent_pose (Pose): geometries Pose object corresponding to the parent frame in the 
            global reference frame
        child_position (Pose): geometries Pose object corresponding to the parent frame in the 
            global reference frame
        
        Returns:
        Transformation: A Transformation object initialized to transform from parent to child frame.
        """
        # Compute the relative translation from parent to child
        translation = child_pose.position._position - parent_pose.position._position

         # Compute the relative rotation quaternion from parent to child
        parent_rotation_inv = Rotation.from_quat(parent_rotation).inv()  # Inverse of parent quaternion
        relative_rotation = Rotation.from_quat(child_rotation).multiply(parent_rotation_inv)

        # Return an instance of the Transformation class with computed translation and rotation
        return cls(translation=translation, rotation=relative_rotation.as_quat())

    @classmethod
    def from_global_poses_backup(cls, parent_position, parent_rotation, child_position, child_rotation):
        """
        Initializes a Transformation object from the global poses of the parent and child frames.
        
        Parameters:
        parent_position (np.array): Position (x, y, z) of the parent frame in global coordinates.
        parent_rotation (np.array): Quaternion (qx, qy, qz, qw) of the parent frame's rotation in global coordinates.
        child_position (np.array): Position (x, y, z) of the child frame in global coordinates.
        child_rotation (np.array): Quaternion (qx, qy, qz, qw) of the child frame's rotation in global coordinates.
        
        Returns:
        Transformation: A Transformation object initialized to transform from parent to child frame.
        """
        # Compute the relative translation from parent to child
        translation = child_position - parent_position

        # Compute the relative rotation from parent to child
        parent_rotation_matrix = Rotation.from_quat(parent_rotation).as_matrix()
        child_rotation_matrix = Rotation.from_quat(child_rotation).as_matrix()
        
        # The relative rotation is the child rotation relative to the parent
        relative_rotation_matrix = child_rotation_matrix.dot(np.linalg.inv(parent_rotation_matrix))

        # Convert the relative rotation matrix back to a quaternion
        relative_rotation = Rotation.from_matrix(relative_rotation_matrix).as_quat()

        # Return an instance of the Transformation class with computed translation and rotation
        return cls(translation=translation, rotation=relative_rotation)

    def __repr__(self):
        """Returns a string representation of the Transformation object."""
        return f"Transformation(translation={self.translation}, rotation={self.rotation})"

# Example usage:
if __name__ == "__main__":
    # Define global pose for parent and child
    parent_position = np.array([0, 0, 0])  # Parent frame position (x, y, z)
    parent_rotation = np.array([0, 0, 0, 1])  # Parent frame quaternion (qx, qy, qz, qw)
    
    child_position = np.array([1, 2, 3])  # Child frame position (x, y, z)
    child_rotation = np.array([0.7071, 0, 0, 0.7071])  # Child frame quaternion (qx, qy, qz, qw) for 90-degree rotation
    
    # Create Transformation object using global poses
    transform = Transformation.from_global_poses(parent_position, parent_rotation, child_position, child_rotation)
    
    # Print the transformation object
    print("Transformation from parent to child:", transform)
