import math
import unittest
from dataclasses import replace

from robotic_arm.application.default_arm_model import create_default_arm_model
from robotic_arm.domain.arm_geometry import ArmGeometry
from robotic_arm.domain.arm_kinematics import ArmKinematics
from robotic_arm.domain.camera.arm_position import ArmPosition
from robotic_arm.domain.arm_space import ArmWaypoint
from robotic_arm.domain.exceptions.errors import JointLimitException, UnreachablePointException
from robotic_arm.domain.gripper_state import GripperState
from robotic_arm.domain.servo_calibration import ServoCalibration

OPEN, CLOSED = GripperState.OPEN, GripperState.CLOSED
REACHABLE = ArmWaypoint(150.0, 0.0, 10.0)

# Poses of the earlier implementation that physically drove this arm (experimental references, not exact IK)
KNOWN_GRASP_POSE = (90, 60, 50, 70, 90, 30)
KNOWN_DROP_POSE = (170, 80, 70, 80, 90, 120)


def distance(a: ArmWaypoint, b: ArmWaypoint) -> float:
    return math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))


class KinematicsTestCase(unittest.TestCase):
    def setUp(self) -> None:
        self.model = create_default_arm_model()
        self.kinematics = ArmKinematics(self.model)

    def kinematics_with(self, **changes) -> ArmKinematics:
        return ArmKinematics(replace(self.model, **changes))


class HomeTests(KinematicsTestCase):
    def test_repose_is_still_90_everywhere(self) -> None:
        self.assertEqual(ArmPosition.repose().angles, (90, 90, 90, 90, 90, 90))

    def test_home_shape_under_the_current_interpretation(self) -> None:
        """Documents what HOME should look like if the joint conventions are right: upper arm vertical,
        forearm horizontal pointing forward, tool horizontal. If the real arm differs, recalibrate."""
        tool = self.kinematics.forward(ArmPosition.repose(), OPEN)
        self.assertAlmostEqual(tool.pitch_deg, 0.0, places=6)
        self.assertAlmostEqual(tool.tcp.x, 86.0 + 122.0, places=6)
        self.assertAlmostEqual(tool.tcp.z, 130.0 + 127.0, places=6)


class ReachablePointTests(KinematicsTestCase):
    def test_reachable_point_gives_a_valid_position(self) -> None:
        position = self.kinematics.inverse(REACHABLE, OPEN)
        self.assertIsInstance(position, ArmPosition)
        self.assertEqual(len(position.angles), 6)
        self.assertTrue(all(isinstance(angle, int) and 0 <= angle <= 180 for angle in position.angles))
        self.assertEqual(position.angles[0], 90)  # straight ahead
        self.assertEqual(position.angles[4], 90)  # wrist roll kept fixed
        self.assertEqual(position.angles[5], 90)  # gripper open

    def test_gripper_state_changes_servo_6_and_the_tool_length(self) -> None:
        opened = self.kinematics.inverse(REACHABLE, OPEN)
        closed = self.kinematics.inverse(REACHABLE, CLOSED)
        self.assertEqual((opened.angles[5], closed.angles[5]), (90, 180))
        self.assertNotEqual(opened.angles[1:4], closed.angles[1:4])  # a longer tool moves the wrist target

    def test_the_elbow_up_configuration_is_used(self) -> None:
        self.assertLess(self.kinematics.inverse(REACHABLE, OPEN).angles[2], 180)

    def test_falls_back_to_the_other_elbow_configuration_when_only_it_fits(self) -> None:
        only_elbow_down = self.kinematics_with(
            shoulder=ServoCalibration(offset=0.0, direction=1, min_angle=0.0, max_angle=180.0),
            elbow=ServoCalibration(offset=180.0, direction=1, min_angle=0.0, max_angle=180.0),
            wrist=ServoCalibration(offset=90.0, direction=1, min_angle=0.0, max_angle=180.0),
        )
        position = only_elbow_down.inverse(REACHABLE, OPEN)
        tool = only_elbow_down.forward(position, OPEN)
        self.assertLess(distance(tool.tcp, REACHABLE), 5.0)

    def test_inverse_then_forward_lands_near_the_waypoint(self) -> None:
        for waypoint, gripper in [
            (ArmWaypoint(150, 0, 10), OPEN),
            (ArmWaypoint(150, 0, 80), OPEN),
            (ArmWaypoint(120, 20, 40), CLOSED),
            (ArmWaypoint(140, -30, 10), OPEN),
            (ArmWaypoint(130, 30, 30), CLOSED),
        ]:
            with self.subTest(waypoint=waypoint, gripper=gripper):
                tool = self.kinematics.forward(self.kinematics.inverse(waypoint, gripper), gripper)
                self.assertLess(distance(tool.tcp, waypoint), 5.0)  # whole-degree servo resolution
                self.assertAlmostEqual(tool.pitch_deg, -90.0, delta=1.5)  # whole-degree rounding resolution


class RejectionTests(KinematicsTestCase):
    def test_point_beyond_the_reach_is_rejected(self) -> None:
        for waypoint in (ArmWaypoint(400, 0, 50), ArmWaypoint(0, 300, 50), ArmWaypoint(250, 250, 50)):
            with self.subTest(waypoint=waypoint):
                with self.assertRaises(UnreachablePointException):
                    self.kinematics.inverse(waypoint, OPEN)

    def test_point_too_close_to_the_base_is_rejected(self) -> None:
        # Distance between shoulder and wrist < abs(upper - fore) = abs(127 - 86) = 41mm
        # If waypoint has wrist distance < 41mm from shoulder:
        with self.assertRaises(UnreachablePointException):
            self.kinematics.inverse(ArmWaypoint(1.0, 0.0, 130.0 + 122.0), OPEN)

    def test_point_on_the_base_axis_is_rejected(self) -> None:
        with self.assertRaises(UnreachablePointException):
            self.kinematics.inverse(ArmWaypoint(0.0, 0.0, 50.0), OPEN)

    def test_point_below_the_safety_floor_is_rejected(self) -> None:
        with self.assertRaises(UnreachablePointException) as context:
            self.kinematics.inverse(ArmWaypoint(150.0, 0.0, -5.0), OPEN)
        self.assertIn("altura mínima", str(context.exception))

    def test_angle_outside_the_servo_limits_is_rejected_not_clamped(self) -> None:
        # REACHABLE shoulder angle is around 36°, narrowing range to [50°, 90°] raises JointLimitException
        narrowed = self.kinematics_with(shoulder=ServoCalibration(offset=0.0, direction=1, min_angle=50.0, max_angle=90.0))
        with self.assertRaises(JointLimitException) as context:
            narrowed.inverse(REACHABLE, OPEN)
        self.assertIn("servo 2", str(context.exception))

    def test_limit_violation_and_out_of_workspace_are_different_errors(self) -> None:
        narrowed = self.kinematics_with(shoulder=ServoCalibration(offset=0.0, direction=1, min_angle=50.0, max_angle=90.0))
        with self.assertRaises(JointLimitException) as limit:
            narrowed.inverse(REACHABLE, OPEN)
        self.assertNotIsInstance(limit.exception, UnreachablePointException)
        with self.assertRaises(UnreachablePointException) as reach:
            narrowed.inverse(ArmWaypoint(500, 0, 50), OPEN)
        self.assertNotIsInstance(reach.exception, JointLimitException)

    def test_base_yaw_beyond_the_servo_range_is_rejected(self) -> None:
        with self.assertRaises(JointLimitException):  # directly behind the arm needs a yaw of 180 degrees
            self.kinematics.inverse(ArmWaypoint(-150.0, 0.0, 10.0), OPEN)


class ConfigurationTests(KinematicsTestCase):
    def test_offset_shifts_the_servo_angle(self) -> None:
        baseline = self.kinematics.inverse(REACHABLE, OPEN).angles
        shifted = self.kinematics_with(shoulder=ServoCalibration(offset=10.0, direction=1)).inverse(REACHABLE, OPEN).angles
        self.assertEqual(shifted[1], baseline[1] + 10)
        self.assertEqual(shifted[0], baseline[0])
        self.assertEqual(shifted[2:], baseline[2:])

    def test_direction_inverts_the_relation(self) -> None:
        waypoint = ArmWaypoint(100.0, 50.0, 10.0)
        baseline = self.kinematics.inverse(waypoint, OPEN).angles
        flipped = self.kinematics_with(
            base=ServoCalibration(offset=90.0, direction=-1),
            shoulder=ServoCalibration(offset=180.0, direction=-1),
        ).inverse(waypoint, OPEN).angles
        self.assertEqual(flipped[0], 180 - baseline[0])  # 90 - yaw instead of 90 + yaw
        self.assertEqual(flipped[1], 180 - baseline[1])  # 180 - q1 instead of q1
        self.assertEqual(flipped[2:], baseline[2:])

    def test_servo_calibration_round_trip(self) -> None:
        calibration = ServoCalibration(offset=37.0, direction=-1)
        self.assertAlmostEqual(calibration.to_mathematical(calibration.to_servo(12.5)), 12.5)

    def test_invalid_calibrations_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            ServoCalibration(offset=0.0, direction=2)
        with self.assertRaises(ValueError):
            ServoCalibration(offset=0.0, min_angle=100.0, max_angle=50.0)
        with self.assertRaises(ValueError):
            ServoCalibration(offset=0.0, max_angle=200.0)
        with self.assertRaises(ValueError):
            ArmGeometry(67.0, 0.0, 110.0, 120.0, 170.0)


class YawAndSymmetryTests(KinematicsTestCase):
    def test_base_yaw_follows_the_coordinate_system(self) -> None:
        servo = lambda x, y: self.kinematics.inverse(ArmWaypoint(x, y, 10.0), OPEN).angles[0]
        self.assertEqual(servo(150, 0), 90)       # +X forward
        self.assertGreater(servo(120, 30), 90)    # +Y left (base direction +1 within [50, 125])
        self.assertLess(servo(120, -30), 90)
        self.assertEqual(servo(100, 57.735), 120) # 30 degrees to the left

    def test_mirrored_points_give_mirrored_base_and_equal_arm_angles(self) -> None:
        left = self.kinematics.inverse(ArmWaypoint(120.0, 40.0, 20.0), OPEN).angles
        right = self.kinematics.inverse(ArmWaypoint(120.0, -40.0, 20.0), OPEN).angles
        self.assertEqual(left[0] + right[0], 180)
        self.assertEqual(left[1:], right[1:])


class KnownPoseTests(KinematicsTestCase):
    """The poses below validate forward and inverse consistency with the real arm model."""

    def test_forward_then_inverse_consistency_with_vertical_tool(self) -> None:
        test_pose = (90, 45, 90, 45, 90, 90)
        tool = self.kinematics.forward(ArmPosition(test_pose), OPEN)
        self.assertAlmostEqual(tool.pitch_deg, -90.0, delta=1.0)
        self.assertAlmostEqual(tool.tcp.y, 0.0, delta=1e-6)
        recovered = self.kinematics.inverse(tool.tcp, OPEN).angles
        self.assertEqual(recovered, test_pose)


if __name__ == "__main__":
    unittest.main()