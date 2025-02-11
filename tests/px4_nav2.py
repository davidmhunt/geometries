#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation

from geometries.pose.pose import Pose
from geometries.pose.position import Position
from geometries.pose.orientation import Orientation

# ---------------------------------------------------------------------------
# Dummy PX4 Odometry message (mimics PX4 REP 147 format in NED)
# ---------------------------------------------------------------------------
class DummyPx4Odometry:
    def __init__(self):
        self.timestamp = 1738778388999184
        self.timestamp_sample = 1738778388999184
        self.pose_frame = 1  # e.g., NED
        # Position in NED: [north, east, down]
        self.position = [-0.24693962931632996,
                         0.02038169465959072,
                         0.4901514947414398]
        # Quaternion in [w, x, y, z] order (rotation from FRD -> NED)
        self.q = [0.6571592092514038,
                  0.010305441915988922,
                  0.027071477845311165,
                  0.7531951069831848]
        self.velocity_frame = 1  # e.g., NED
        # Linear velocity in NED coordinates.
        self.velocity = [-0.07290629297494888,
                         0.04737717658281326,
                         0.19108231365680695]
        # Angular velocity (roll, pitch, yaw rates) in body FRD.
        self.angular_velocity = [0.0003648111887741834,
                                 4.621630068868399e-05,
                                 -4.517537308856845e-05]
        self.position_variance = [0.043032027781009674,
                                  0.04309243708848953,
                                  0.07466155290603638]
        self.orientation_variance = [4.042432192363776e-05,
                                     4.439154145075008e-05,
                                     0.003111481899395585]
        self.velocity_variance = [0.008963913656771183,
                                  0.008973918855190277,
                                  0.004697065334767103]
        self.reset_counter = 13
        self.quality = 0

    def __repr__(self):
        return (f"DummyPx4Odometry(timestamp={self.timestamp}, "
                f"position={self.position}, q={self.q},\n"
                f"  velocity={self.velocity}, angular_velocity={self.angular_velocity})")


# ---------------------------------------------------------------------------
# Dummy nav2 Odometry message (mimics nav_msgs/msg/Odometry in ENU)
# ---------------------------------------------------------------------------
class DummyNav2Odometry:
    def __init__(self):
        # Header
        self.header = type("Header", (), {})()
        self.header.seq = 0
        self.header.stamp = type("Stamp", (), {})()
        self.header.stamp.sec = 0
        self.header.stamp.nanosec = 0
        self.header.frame_id = "odom"

        self.child_frame_id = "base_link"

        # Pose (PoseWithCovariance)
        self.pose = type("PoseWithCovariance", (), {})()
        self.pose.pose = type("Pose", (), {})()
        self.pose.pose.position = type("Point", (), {})()
        self.pose.pose.orientation = type("Quaternion", (), {})()
        self.pose.covariance = [0.0] * 36

        # Twist (TwistWithCovariance)
        self.twist = type("TwistWithCovariance", (), {})()
        self.twist.twist = type("Twist", (), {})()
        self.twist.twist.linear = type("Vector3", (), {})()
        self.twist.twist.angular = type("Vector3", (), {})()
        self.twist.covariance = [0.0] * 36

    def __repr__(self):
        pos = (self.pose.pose.position.x,
               self.pose.pose.position.y,
               self.pose.pose.position.z)
        orient = (self.pose.pose.orientation.x,
                  self.pose.pose.orientation.y,
                  self.pose.pose.orientation.z,
                  self.pose.pose.orientation.w)
        lin_twist = (self.twist.twist.linear.x,
                     self.twist.twist.linear.y,
                     self.twist.twist.linear.z)
        ang_twist = (self.twist.twist.angular.x,
                     self.twist.twist.angular.y,
                     self.twist.twist.angular.z)
        return (f"DummyNav2Odometry(header.frame_id={self.header.frame_id},\n"
                f"  position={pos},\n"
                f"  orientation={orient},\n"
                f"  linear_twist={lin_twist}, angular_twist={ang_twist})")


# ---------------------------------------------------------------------------
# Conversion Functions
# ---------------------------------------------------------------------------
def convert_px4_to_nav2(px4_odom):
    """
    Convert a PX4 odometry message (NED, [w,x,y,z] quaternion) to
    a nav2 odometry message (ENU).
    Conversion chain for pose: NED --> (flu_from_ned) FLU --> (flu_to_enu) ENU.
    For twist, the same rotations are applied to the linear and angular velocity vectors.
    """
    # --- Pose Conversion ---
    # Reorder quaternion from [w,x,y,z] to [x,y,z,w] for our Pose routines.
    px4_q = px4_odom.q
    quat_ned = [px4_q[1], px4_q[2], px4_q[3], px4_q[0]]

    # Create a Pose in NED.
    pos_ned = Position(px4_odom.position[0],
                       px4_odom.position[1],
                       px4_odom.position[2])
    orient_ned = Orientation(quat_ned[0],
                             quat_ned[1],
                             quat_ned[2],
                             quat_ned[3])
    pose_ned = Pose(pos_ned, orient_ned)

    # Convert NED to FLU then FLU to ENU.
    pose_flu = pose_ned.flu_from_ned()
    pose_enu = pose_flu.flu_to_enu()

    # Populate a dummy nav2 message with the converted pose.
    nav2_odom = DummyNav2Odometry()
    nav2_odom.pose.pose.position.x = pose_enu.position.x
    nav2_odom.pose.pose.position.y = pose_enu.position.y
    nav2_odom.pose.pose.position.z = pose_enu.position.z
    nav2_odom.pose.pose.orientation.x = pose_enu.orientation.qx
    nav2_odom.pose.pose.orientation.y = pose_enu.orientation.qy
    nav2_odom.pose.pose.orientation.z = pose_enu.orientation.qz
    nav2_odom.pose.pose.orientation.w = pose_enu.orientation.qw

    # --- Twist Conversion (PX4 -> nav2) ---
    # Define the same rotation chain for velocities:
    # 1. NED to FLU: rotate about x-axis 180 degrees.
    R_ned_to_flu = Rotation.from_euler('x', 180, degrees=True)
    # 2. FLU to ENU: rotate about z-axis -90 degrees.
    R_flu_to_enu = Rotation.from_euler('z', -90, degrees=True)

    # Convert linear velocity:
    v_ned = np.array(px4_odom.velocity)
    v_flu = R_ned_to_flu.apply(v_ned)
    v_enu = R_flu_to_enu.apply(v_flu)
    nav2_odom.twist.twist.linear.x = v_enu[0]
    nav2_odom.twist.twist.linear.y = v_enu[1]
    nav2_odom.twist.twist.linear.z = v_enu[2]

    # Convert angular velocity:
    omega_px4 = np.array(px4_odom.angular_velocity)
    omega_flu = R_ned_to_flu.apply(omega_px4)
    omega_enu = R_flu_to_enu.apply(omega_flu)
    nav2_odom.twist.twist.angular.x = omega_enu[0]
    nav2_odom.twist.twist.angular.y = omega_enu[1]
    nav2_odom.twist.twist.angular.z = omega_enu[2]

    return nav2_odom


def convert_nav2_to_px4(nav2_odom):
    """
    Convert a nav2 odometry message (ENU) to a PX4 odometry message (NED, [w,x,y,z]).
    Conversion chain for pose: ENU --> (flu_from_enu) FLU --> (flu_to_ned) NED.
    For twist, the same rotations are applied to the linear and angular velocity vectors.
    """
    # --- Pose Conversion ---
    pos_enu = Position(nav2_odom.pose.pose.position.x,
                       nav2_odom.pose.pose.position.y,
                       nav2_odom.pose.pose.position.z)
    orient_enu = Orientation(nav2_odom.pose.pose.orientation.x,
                             nav2_odom.pose.pose.orientation.y,
                             nav2_odom.pose.pose.orientation.z,
                             nav2_odom.pose.pose.orientation.w)
    pose_enu = Pose(pos_enu, orient_enu)

    # Convert ENU to FLU then FLU to NED.
    pose_flu = pose_enu.flu_from_enu()
    pose_ned = pose_flu.flu_to_ned()

    # Populate a dummy PX4 message.
    px4_odom = DummyPx4Odometry()
    px4_odom.timestamp = 123456789  # example timestamp
    px4_odom.timestamp_sample = 123456789
    px4_odom.pose_frame = 1  # NED frame

    # PX4 position is in NED.
    px4_odom.position = [pose_ned.position.x,
                         pose_ned.position.y,
                         pose_ned.position.z]
    # Reorder quaternion from [qx, qy, qz, qw] to [w, x, y, z].
    q = pose_ned.orientation
    px4_odom.q = [q.qw, q.qx, q.qy, q.qz]

    # --- Twist Conversion (nav2 -> PX4) ---
    # Define rotation chain for velocities from ENU to NED:
    # 1. ENU to FLU: inverse of FLU to ENU. (Rotation.from_euler('z', -90)⁻¹ = Rotation.from_euler('z', 90))
    R_enu_to_flu = Rotation.from_euler('z', 90, degrees=True)
    # 2. FLU to NED: rotate about x-axis 180 degrees (self-inverse).
    R_flu_to_ned = Rotation.from_euler('x', 180, degrees=True)

    # Convert linear velocity:
    v_enu = np.array([nav2_odom.twist.twist.linear.x,
                      nav2_odom.twist.twist.linear.y,
                      nav2_odom.twist.twist.linear.z])
    v_flu = R_enu_to_flu.apply(v_enu)
    v_ned = R_flu_to_ned.apply(v_flu)
    px4_odom.velocity = [v_ned[0], v_ned[1], v_ned[2]]

    # Convert angular velocity:
    omega_enu = np.array([nav2_odom.twist.twist.angular.x,
                          nav2_odom.twist.twist.angular.y,
                          nav2_odom.twist.twist.angular.z])
    omega_flu = R_enu_to_flu.apply(omega_enu)
    omega_ned = R_flu_to_ned.apply(omega_flu)
    px4_odom.angular_velocity = [omega_ned[0], omega_ned[1], omega_ned[2]]

    return px4_odom


# ---------------------------------------------------------------------------
# Main: Demonstrate Round-Trip Conversions
# ---------------------------------------------------------------------------
if __name__ == "__main__":

    print("=== PX4 -> nav2 -> PX4 roundtrip ===")
    # 1. Start with a dummy PX4 message.
    px4_orig = DummyPx4Odometry()
    print("Original PX4 message:")
    print(px4_orig)

    # 2. Convert from PX4 (NED) to nav2 (ENU).
    nav2_from_px4 = convert_px4_to_nav2(px4_orig)
    print("\nConverted to nav2 message:")
    print(nav2_from_px4)

    # 3. Convert back from nav2 (ENU) to PX4 (NED).
    px4_from_nav2 = convert_nav2_to_px4(nav2_from_px4)
    print("\nConverted back to PX4 message:")
    print(px4_from_nav2)


    print("\n=== nav2 -> PX4 -> nav2 roundtrip ===")
    # 1. Start with a dummy nav2 message.
    nav2_orig = DummyNav2Odometry()
    # Set some sample ENU values for pose.
    nav2_orig.pose.pose.position.x = 1.234
    nav2_orig.pose.pose.position.y = 2.345
    nav2_orig.pose.pose.position.z = 0.567
    nav2_orig.pose.pose.orientation.x = 0.0
    nav2_orig.pose.pose.orientation.y = 0.0
    nav2_orig.pose.pose.orientation.z = 0.0
    nav2_orig.pose.pose.orientation.w = 1.0
    # Set some sample twist values for nav2 (in ENU).
    nav2_orig.twist.twist.linear.x = 0.5
    nav2_orig.twist.twist.linear.y = -0.2
    nav2_orig.twist.twist.linear.z = 0.1
    nav2_orig.twist.twist.angular.x = 0.01
    nav2_orig.twist.twist.angular.y = -0.02
    nav2_orig.twist.twist.angular.z = 0.03
    print("\nOriginal nav2 message:")
    print(nav2_orig)

    # 2. Convert from nav2 (ENU) to PX4 (NED).
    px4_from_nav2 = convert_nav2_to_px4(nav2_orig)
    print("\nConverted to PX4 message:")
    print(px4_from_nav2)

    # 3. Convert back from PX4 (NED) to nav2 (ENU).
    nav2_from_px4 = convert_px4_to_nav2(px4_from_nav2)
    print("\nConverted back to nav2 message:")
    print(nav2_from_px4)
