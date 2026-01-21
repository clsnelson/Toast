from enum import Enum, auto
from typing import Final

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import RobotBase


class Constants:
    tuningMode: Final[bool] = False

    class Mode(Enum):
        # Running on a real robot.
        REAL = auto(),

        # Running a physics simulator.
        SIM = auto(),

        # Replaying from a log file.
        REPLAY = auto()

    simMode: Final[Mode] = Mode.SIM
    currentMode: Final[Mode] = Mode.REAL if RobotBase.isReal() else simMode

    FIELD_LAYOUT: Final[AprilTagFieldLayout] = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

    class AutoAlignConstants:
        translationkP = 9.0
        translationkI = 0.0
        translationkD = 0.1

        headingkP = 5.0
        headingkI = 0.0
        headingkD = 0.4
