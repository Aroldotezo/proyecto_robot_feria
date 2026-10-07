import unittest

from robotic_arm.adapters.in_memory_profile_repository import InMemoryProfileRepository
from robotic_arm.application.calibration_service import CalibrationService
from robotic_arm.application.default_arm_model import create_default_arm_model
from robotic_arm.application.default_profile import create_default_profile
from robotic_arm.application.motion_planner import MotionPlanner
from robotic_arm.domain.arm_kinematics import ArmKinematics
from robotic_arm.domain.arm_space import ArmPoint
from robotic_arm.domain.geometry import NormalizedPoint
from robotic_arm.domain.gripper_state import GripperState
from robotic_arm.domain.motion_step import MotionAction

# Made-up calibration that keeps the work area where the default heights are feasible
# (with a vertical tool and the default model, a radius of about 65-155 mm; x forward, y left, mm)
REFERENCE_POINTS = (
    (NormalizedPoint(0.1, 0.1), ArmPoint(70.0, -70.0)),
    (NormalizedPoint(0.9, 0.1), ArmPoint(70.0, 70.0)),
    (NormalizedPoint(0.9, 0.9), ArmPoint(140.0, 70.0)),
    (NormalizedPoint(0.1, 0.9), ArmPoint(140.0, -70.0)),
)


class PickAndPlaceSequenceTests(unittest.TestCase):
    def test_every_step_of_a_planned_sequence_can_be_solved_by_the_inverse_kinematics(self) -> None:
        repository = InMemoryProfileRepository([create_default_profile()])
        calibration = CalibrationService(repository, "Organic sorting")
        for image, arm in REFERENCE_POINTS:
            calibration.add_reference_point(image, arm)
        planner = MotionPlanner(calibration)
        kinematics = ArmKinematics(create_default_arm_model())

        for category in ("organic", "inorganic", "defective"):
            with self.subTest(category=category):
                gripper = GripperState.OPEN
                for step in planner.plan_pick_and_place(NormalizedPoint(0.5, 0.25), category):
                    if step.action is MotionAction.CLOSE_GRIPPER:
                        gripper = GripperState.CLOSED
                    elif step.action is MotionAction.OPEN_GRIPPER:
                        gripper = GripperState.OPEN
                    elif step.action is MotionAction.MOVE_TO:
                        position = kinematics.inverse(step.waypoint, gripper)  # raises if it cannot be solved
                        self.assertEqual(len(position.angles), 6)


if __name__ == "__main__":
    unittest.main()