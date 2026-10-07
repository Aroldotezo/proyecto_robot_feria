import unittest
from dataclasses import replace

from robotic_arm.adapters.fake_arm import FakeArm
from robotic_arm.application.arm_service import ArmService
from robotic_arm.application.default_arm_model import create_default_arm_model
from robotic_arm.domain.arm_kinematics import ArmKinematics
from robotic_arm.domain.camera.arm_position import ArmPosition
from robotic_arm.domain.arm_space import ArmWaypoint
from robotic_arm.domain.exceptions.errors import DisconnectedArmException, JointLimitException, UnreachablePointException
from robotic_arm.domain.gripper_state import GripperState
from robotic_arm.domain.servo_calibration import ServoCalibration
from robotic_arm.ports.arm_controller import ArmController

REACHABLE = ArmWaypoint(150.0, 0.0, 10.0)


class RecordingArm(ArmController):
    """Stands in for SerialArm: records exactly what ArmService hands to the controller."""

    created: list["RecordingArm"] = []

    def __init__(self, port: str) -> None:
        self.moves: list[object] = []
        self._connected = False
        RecordingArm.created.append(self)

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def assign_speed(self, speed: int) -> None:
        pass

    def move_to(self, pose: ArmPosition) -> None:
        self.moves.append(pose)


class ForbiddenKinematics(ArmKinematics):
    def inverse(self, waypoint, gripper=GripperState.OPEN):
        raise AssertionError("HOME must never go through inverse kinematics")


class ArmServiceWaypointTests(unittest.TestCase):
    def setUp(self) -> None:
        RecordingArm.created.clear()
        self.model = create_default_arm_model()
        self.kinematics = ArmKinematics(self.model)

    def connected_service(self, kinematics: ArmKinematics | None = None) -> tuple[ArmService, RecordingArm]:
        service = ArmService(arm_factory=RecordingArm, kinematics=kinematics or self.kinematics)
        service.connect("TEST", speed=30)
        return service, RecordingArm.created[-1]

    def test_home_is_sent_as_is_and_never_uses_inverse_kinematics(self) -> None:
        service, arm = self.connected_service(ForbiddenKinematics(self.model))
        service.move_to_rest()
        self.assertEqual(len(arm.moves), 1)
        self.assertEqual(arm.moves[0].angles, (90, 90, 90, 90, 90, 90))

    def test_a_waypoint_sends_exactly_one_arm_position(self) -> None:
        service, arm = self.connected_service()
        pose = service.move_to_waypoint(REACHABLE, GripperState.OPEN)
        self.assertEqual(len(arm.moves), 1)
        self.assertIsInstance(arm.moves[0], ArmPosition)
        self.assertEqual(arm.moves[0], pose)
        self.assertEqual(pose, self.kinematics.inverse(REACHABLE, GripperState.OPEN))

    def test_a_point_outside_the_workspace_sends_nothing(self) -> None:
        service, arm = self.connected_service()
        with self.assertRaises(UnreachablePointException):
            service.move_to_waypoint(ArmWaypoint(500.0, 0.0, 50.0))
        self.assertEqual(arm.moves, [])

    def test_a_joint_limit_violation_sends_nothing(self) -> None:
        narrowed = ArmKinematics(replace(self.model, shoulder=ServoCalibration(offset=0.0, min_angle=70.0)))
        service, arm = self.connected_service(narrowed)
        with self.assertRaises(JointLimitException):
            service.move_to_waypoint(REACHABLE)
        self.assertEqual(arm.moves, [])

    def test_compute_pose_is_a_dry_run(self) -> None:
        service = ArmService(arm_factory=RecordingArm, kinematics=self.kinematics)  # not even connected
        self.assertEqual(service.compute_pose(REACHABLE).angles[0], 90)
        self.assertEqual(RecordingArm.created, [])

    def test_moving_requires_a_connection(self) -> None:
        service = ArmService(arm_factory=RecordingArm, kinematics=self.kinematics)
        with self.assertRaises(DisconnectedArmException):
            service.move_to_waypoint(REACHABLE)

    def test_the_fake_arm_keeps_working(self) -> None:
        service = ArmService(arm_factory=lambda port: FakeArm(port, move_delay=0.0), kinematics=self.kinematics)
        service.connect("SIMULATED", speed=30)
        service.move_to_rest()
        self.assertEqual(service.move_to_waypoint(REACHABLE).angles[0], 90)
        service.disconnect()


if __name__ == "__main__":
    unittest.main()