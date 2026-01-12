from dataclasses import dataclass
from typing import Final, List

from pathplannerlib.config import RobotConfig, ModuleConfig
from wpimath.system.plant import DCMotor
from wpimath.units import *


class DriveConstants:

    @dataclass
    class ModuleConfig:
        steerMotorId: int
        driveMotorId: int
        encoderId: int
        encoderOffsetRotations: float
        isDriveInverted: bool
        isSteerInverted: bool
        isEncoderInverted: bool

    odomFrequency: Final[hertz] = 250

    trackWidthMeters = inchesToMeters(24.0)
    driveBaseRadius: Final[meters] = trackWidthMeters / math.sqrt(2) # Assumes square drivetrain.

    maxLinearSpeed: Final[meters_per_second] = 4.2
    maxAngularSpeed: Final[radians_per_second] = 4.2 / driveBaseRadius

    steerDeadband: Final[degrees] = 0.3

    wheelRadius: Final[inches] = 2

    # PathPlanner config constants
    robotMass: Final[kilograms] = 74.088
    robotMOI: Final[kilogram_square_meters] = 6.883
    wheelCOF: Final[float] = 1.2

    _pathPlannerConfig = None

    @classmethod
    def getPathPlannerConfig(cls, translations=None) -> RobotConfig:
        # Circular imports be damned
        if cls._pathPlannerConfig is None:
            cls._pathPlannerConfig = RobotConfig(
                cls.robotMass,
                cls.robotMOI,
                ModuleConfig(
                    inchesToMeters(cls.wheelRadius),
                    cls.maxLinearSpeed,
                    cls.wheelCOF,
                    DCMotor.krakenX60FOC(1).withReduction(6.746031746031747),
                    80,
                    1),
                translations
            )
        return cls._pathPlannerConfig

    moduleConfigs: Final[List[ModuleConfig]] = [
        # Front Left
        ModuleConfig(
            3, 7, 7, 0.155029296875, True, False, False
        ),
        # Front Right
        ModuleConfig(
            1, 5, 5, -0.27978515625, True, False, False
        ),
        # Back Left
        ModuleConfig(
            4, 8, 8, -0.241943359375, True, False, False
        ),
        # Back Right
        ModuleConfig(
            2, 6, 6, -0.06494140625, True, False, False
        )
    ]

    class PigeonConstants:
        id: Final[int] = 9