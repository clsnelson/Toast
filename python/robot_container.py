import math
import os.path

import commands2.button
from commands2 import cmd, Command
from commands2.button import CommandXboxController
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.util import FlippingUtil
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from wpilib import getDeployDirectory
from wpimath.geometry import Pose2d, Rotation2d

import commands
from commands import fieldRelative
from constants import Constants
from subsystems.drive import Drive, DriveConstants
from subsystems.drive.gyro import GyroIOPigeon2, GyroIOSim
from subsystems.drive.module import ModuleIOTalonFX, ModuleIOSim


class RobotContainer:
    def __init__(self) -> None:
        self._driver = CommandXboxController(0)

        match Constants.currentMode:
            case Constants.Mode.REAL:
                # Real robot, instantiate hardware IO implementations
                self._drivetrain = Drive(
                    GyroIOPigeon2(),
                    ModuleIOTalonFX(DriveConstants.moduleConfigs[0]),
                    ModuleIOTalonFX(DriveConstants.moduleConfigs[1]),
                    ModuleIOTalonFX(DriveConstants.moduleConfigs[2]),
                    ModuleIOTalonFX(DriveConstants.moduleConfigs[3]),
                    lambda _: None
                )
            case Constants.Mode.SIM:
                # Sim robot, instantiate physics sim IO implementations
                # ... IF WE HAD ONE
                self._drivetrain = Drive(
                    GyroIOSim(),
                    ModuleIOSim(DriveConstants.moduleConfigs[0]),
                    ModuleIOSim(DriveConstants.moduleConfigs[1]),
                    ModuleIOSim(DriveConstants.moduleConfigs[2]),
                    ModuleIOSim(DriveConstants.moduleConfigs[3]),
                    lambda _: None
                )

        # Auto chooser
        self.autoChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser("Selected Auto")

        auto_files = os.listdir(os.path.join(getDeployDirectory(), "pathplanner", "autos"))
        for file in auto_files:
            file = file.removesuffix(".auto")
            self.autoChooser.addOption(file, PathPlannerAuto(file, False))
            self.autoChooser.addOption(f"{file} (Mirrored)", PathPlannerAuto(file, True))
        self.autoChooser.setDefaultOption("None", cmd.none())
        self.autoChooser.addOption("Basic Leave",
        self._drivetrain.run(lambda: commands.robotRelative(self._drivetrain, lambda: 0.25, lambda: 0, lambda: 0)).withTimeout(1.0))

        self.setupControllerBindings()

    def readyRobotForMatch(self) -> None:
        auto = self.autoChooser.getSelected()
        if isinstance(auto, PathPlannerAuto):
            if AutoBuilder.shouldFlip():
                self._drivetrain.setPose(FlippingUtil.flipFieldPose(auto._startingPose))
            else:
                self._drivetrain.setPose(auto._startingPose)

    def setupControllerBindings(self) -> None:
        # DRIVE CONTROLLER
        self._drivetrain.setDefaultCommand(fieldRelative(
            self._drivetrain,
            lambda: -self._driver.getLeftY(),
            lambda: -self._driver.getLeftX(),
            lambda: -self._driver.getRightX()
        ))

        # Left bumper: Robot relative
        self._driver.leftBumper().whileTrue(
            commands.robotRelative(
                self._drivetrain,
                lambda: -self._driver.getLeftY(),
                lambda: -self._driver.getLeftX(),
                lambda: -self._driver.getRightX()
            )
        )

        # A: X-brake
        self._driver.a().whileTrue(commands.brakeWithX(self._drivetrain))

        # Left trigger: Align to closest left branch
        self._driver.leftTrigger().whileTrue(
            commands.alignToClosestBranch(
                self._drivetrain,
                commands.BranchSide.LEFT,
                lambda: -self._driver.getLeftY(),
                lambda: -self._driver.getLeftX()
            )
        )

        # Right trigger: Align to closest right branch
        self._driver.rightTrigger().whileTrue(
            commands.alignToClosestBranch(
                self._drivetrain,
                commands.BranchSide.RIGHT,
                lambda: -self._driver.getLeftY(),
                lambda: -self._driver.getLeftX()
            )
        )

        # Reset gyro to 0 when start button is pressed.
        self._driver.start().onTrue(cmd.runOnce(
            lambda: self._drivetrain.setPose(
                Pose2d(self._drivetrain.getPose().translation(), Rotation2d() + Rotation2d(math.pi) if AutoBuilder.shouldFlip() else Rotation2d())
            ), self._drivetrain
        ).ignoringDisable(True))

    def get_autonomous_command(self) -> commands2.Command:
        return self.autoChooser.getSelected()
