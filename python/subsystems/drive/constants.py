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

    # Odometry frequency of the odometry thread. 75hz for roboRIO CAN, 250hz if using a CANivore (CANFD)
    odomFrequency: Final[hertz] = 250

    # Distance between 2 wheels (horizontally or vertically)
    trackWidthMeters = inchesToMeters(24.0)
    driveBaseRadius: Final[meters] = trackWidthMeters / math.sqrt(2) # Assumes square drivetrain.

    # THEORETICAL max free speed of the robot. This is the robot travelling max speed in one direction.
    # Easiest way to measure this is to check a wheel's highest rot/s in Phoenix Tuner then multiply by the wheel's circumference (in meters).
    maxLinearSpeed: Final[meters_per_second] = 4.2
    maxAngularSpeed: Final[radians_per_second] = 4.2 / driveBaseRadius

    # Minimum degree change needed before the steer motors move. 0.3 by default.
    steerDeadband: Final[degrees] = 0.3

    # Wheel radius in inches.
    # Use the characterization test periodically to update this value through the course of an event.
    wheelRadius: Final[inches] = 2

    ### PathPlanner config constants
    # Mass of the robot in kilograms.
    # If you can't physically measure, take CAD estimate and add 10% of robot weight for wires.
    robotMass: Final[kilograms] = 74.088

    # MOI of the robot in kg^2/m. Personally, I take the CAD's estimate (with game pieces) and round up to account for wires.
    robotMOI: Final[kilogram_square_meters] = 6.883

    # Coefficient of friction for tread. This can probably be measured somehow, but 1.2 works well for SDS tread.
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
            3, # Steer motor ID
            7, # Drive motor ID
            7, # Encoder ID
            0.155029296875, # Encoder offset (zero encoders in Phoenix Tuner)
            True, # Invert drive motor (use if driving backwards)
            False, # Invert steer motors (use if steering in reverse)
            False # Invert encoder motors (use if steer motor position isn't working properly)
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