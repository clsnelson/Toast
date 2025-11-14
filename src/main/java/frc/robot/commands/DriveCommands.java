// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import static edu.wpi.first.math.MathUtil.applyDeadband;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.*;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  /** Enum describing which branch side to target */
  public enum BranchSide {
    LEFT,
    RIGHT
  }

  // Predefined reef branch target poses (Blue alliance, mirrored for Red)
  private static final Pose2d[] BLUE_LEFT_BRANCHES = {
    new Pose2d(3.091, 4.181, Rotation2d.kZero),
    new Pose2d(3.656, 2.916, Rotation2d.fromDegrees(60)),
    new Pose2d(5.023, 2.772, Rotation2d.fromDegrees(120)),
    new Pose2d(5.850, 3.851, Rotation2d.k180deg),
    new Pose2d(5.347, 5.134, Rotation2d.fromDegrees(240)),
    new Pose2d(3.932, 5.302, Rotation2d.fromDegrees(300))
  };

  private static final Pose2d[] BLUE_RIGHT_BRANCHES = {
    new Pose2d(3.091, 3.863, Rotation2d.kZero),
    new Pose2d(3.956, 2.748, Rotation2d.fromDegrees(60)),
    new Pose2d(5.323, 2.928, Rotation2d.fromDegrees(120)),
    new Pose2d(5.862, 4.187, Rotation2d.k180deg),
    new Pose2d(5.047, 5.290, Rotation2d.fromDegrees(240)),
    new Pose2d(3.668, 5.110, Rotation2d.fromDegrees(300))
  };

  private static final Pose2d[] RED_LEFT_BRANCHES = mirrorPoses(BLUE_LEFT_BRANCHES);
  private static final Pose2d[] RED_RIGHT_BRANCHES = mirrorPoses(BLUE_RIGHT_BRANCHES);

  private static Pose2d[] mirrorPoses(Pose2d[] blueTargets) {
    Pose2d[] mirrored = new Pose2d[blueTargets.length];
    double fieldLength = Constants.FIELD_LAYOUT.getFieldLength();
    double fieldWidth = Constants.FIELD_LAYOUT.getFieldWidth();
    for (int i = 0; i < blueTargets.length; i++) {
      Pose2d pose = blueTargets[i];
      mirrored[i] =
          new Pose2d(
              fieldLength - pose.getX(),
              fieldWidth - pose.getY(),
              pose.getRotation().plus(Rotation2d.k180deg));
    }
    return mirrored;
  }

  private static Pose2d closestBranch;

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command fieldRelative(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              ChassisSpeeds speeds =
                  getChassisSpeeds(driveSubsystem, omegaSupplier, linearVelocity);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              driveSubsystem.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? driveSubsystem.getRotation().plus(Rotation2d.kPi)
                          : driveSubsystem.getRotation()));
            },
            driveSubsystem)
        .withName("FieldRelativeDrive");
  }

  /**
   * Robot relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command robotRelative(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
              driveSubsystem.runVelocity(
                  getChassisSpeeds(driveSubsystem, omegaSupplier, linearVelocity));
            },
            driveSubsystem)
        .withName("RobotRelativeDrive");
  }

  /** Stops the drive and turns the modules to an X arrangement to resist movement. */
  public static Command brakeWithX(DriveSubsystem driveSubsystem) {
    return Commands.run(driveSubsystem::stopWithX).withName("BrakeX");
  }

  public static Command alignToClosestReefBranch(
      DriveSubsystem driveSubsystem,
      BranchSide branchSide,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

    final PIDController translationController =
        new PIDController(
            Constants.AutoAlignConstants.TRANSLATION_P,
            Constants.AutoAlignConstants.TRANSLATION_I,
            Constants.AutoAlignConstants.TRANSLATION_D);
    final ProfiledPIDController rotationController =
        new ProfiledPIDController(
            Constants.AutoAlignConstants.HEADING_P,
            Constants.AutoAlignConstants.HEADING_I,
            Constants.AutoAlignConstants.HEADING_D,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.runOnce(
            () -> {
              closestBranch = getClosestBranch(driveSubsystem, branchSide);
              Logger.recordOutput("Drive/TargetBranch", closestBranch);
            })
        .andThen(
            Commands.run(
                    () -> {
                      Pose2d currentPose = driveSubsystem.getPose();
                      Rotation2d targetRot = closestBranch.getRotation();

                      // Compute Y error rotated to robot frame
                      double yError =
                          (currentPose.getX() * -targetRot.getSin()
                                  + currentPose.getY() * targetRot.getCos())
                              - (closestBranch.getX() * -targetRot.getSin()
                                  + closestBranch.getY() * targetRot.getCos());

                      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                        yError *= -1.0;
                      }

                      // Rotate robot relative velocity to field-centric
                      double clamped =
                          Math.max(
                              -1.5, Math.min(translationController.calculate(yError, 0.0), 1.5));
                      double velY =
                          applyDeadband(
                              clamped, driveSubsystem.getMaxLinearSpeedMetersPerSec() * 0.01);

                      Translation2d fieldRelativeVelocity =
                          new Translation2d(
                                  xSupplier.getAsDouble() * targetRot.getCos()
                                      + ySupplier.getAsDouble() * targetRot.getSin(),
                                  velY)
                              .rotateBy(targetRot);

                      // Heading PID
                      double rotationVel =
                          rotationController.calculate(
                              currentPose.getRotation().getRadians(), targetRot.getRadians());

                      ChassisSpeeds speeds =
                          new ChassisSpeeds(
                              fieldRelativeVelocity.getX(),
                              fieldRelativeVelocity.getY(),
                              rotationVel);

                      boolean isFlipped =
                          DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red;

                      // Drive field-relative
                      driveSubsystem.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              speeds,
                              isFlipped
                                  ? driveSubsystem.getRotation().plus(Rotation2d.kPi)
                                  : driveSubsystem.getRotation()));
                    },
                    driveSubsystem)
                .withName("AlignToClosestReefBranch")
                // Reset PID controller when command starts
                .beforeStarting(
                    () -> {
                      rotationController.reset(driveSubsystem.getRotation().getRadians());
                      translationController.reset();
                    }));
  }

  private static Pose2d getClosestBranch(DriveSubsystem driveSubsystem, BranchSide branchSide) {
    // Get the closest target pose
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Pose2d[] targets;
    if (alliance == Alliance.Red) {
      targets = (branchSide == BranchSide.LEFT ? RED_LEFT_BRANCHES : RED_RIGHT_BRANCHES);
    } else {
      targets = (branchSide == BranchSide.LEFT ? BLUE_LEFT_BRANCHES : BLUE_RIGHT_BRANCHES);
    }

    return Arrays.stream(targets)
        .min(Comparator.comparingDouble(p -> getDistanceToLine(driveSubsystem.getPose(), p)))
        .orElse(targets[0]);
  }

  /** Compute distance between robot and target branch line */
  private static double getDistanceToLine(Pose2d robot, Pose2d target) {
    double slope = Math.tan(target.getRotation().getRadians());
    double x =
        (robot.getX()
                + slope * slope * target.getX()
                - slope * target.getY()
                + slope * robot.getY())
            / (slope * slope + 1);
    double y = slope * (x - target.getX()) + target.getY();

    Pose2d possible = new Pose2d(x, y, target.getRotation());
    double reefX =
        (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
            ? 4.4735
            : Constants.FIELD_LAYOUT.getFieldLength() - 4.4735;

    if ((target.getX() - reefX <= 0) == (x - reefX <= 0))
      return robot.getTranslation().getDistance(possible.getTranslation());
    return Double.POSITIVE_INFINITY;
  }

  private static ChassisSpeeds getChassisSpeeds(
      DriveSubsystem driveSubsystem, DoubleSupplier omegaSupplier, Translation2d linearVelocity) {
    double omega = applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    // Convert to field relative speeds & send command
    return new ChassisSpeeds(
        linearVelocity.getX() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
        linearVelocity.getY() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
        omega * driveSubsystem.getMaxAngularSpeedRadPerSec());
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      DriveSubsystem driveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      driveSubsystem.getRotation().getRadians(),
                      rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * driveSubsystem.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              driveSubsystem.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? driveSubsystem.getRotation().plus(Rotation2d.kPi)
                          : driveSubsystem.getRotation()));
            },
            driveSubsystem)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(driveSubsystem.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(DriveSubsystem driveSubsystem) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                  velocitySamples.clear();
                  voltageSamples.clear();
                }),

            // Allow modules to orient
            Commands.run(() -> driveSubsystem.runCharacterization(0.0), driveSubsystem)
                .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                    () -> {
                      double voltage = timer.get() * FF_RAMP_RATE;
                      driveSubsystem.runCharacterization(voltage);
                      velocitySamples.add(driveSubsystem.getFFCharacterizationVelocity());
                      voltageSamples.add(voltage);
                    },
                    driveSubsystem)

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      int n = velocitySamples.size();
                      double sumX = 0.0;
                      double sumY = 0.0;
                      double sumXY = 0.0;
                      double sumX2 = 0.0;
                      for (int i = 0; i < n; i++) {
                        sumX += velocitySamples.get(i);
                        sumY += voltageSamples.get(i);
                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                      }
                      double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                      double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                      NumberFormat formatter = new DecimalFormat("#0.00000");
                      System.out.println("********** Drive FF Characterization Results **********");
                      System.out.println("\tkS: " + formatter.format(kS));
                      System.out.println("\tkV: " + formatter.format(kV));
                    }))
        .withName("Feedforward Characterization");
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(DriveSubsystem driveSubsystem) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(() -> limiter.reset(0.0)),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                      double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                      driveSubsystem.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    driveSubsystem)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                      state.positions = driveSubsystem.getWheelRadiusCharacterizationPositions();
                      state.lastAngle = driveSubsystem.getRotation();
                      state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                          var rotation = driveSubsystem.getRotation();
                          state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                          state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                          double[] positions =
                              driveSubsystem.getWheelRadiusCharacterizationPositions();
                          double wheelDelta = 0.0;
                          for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                          }
                          double wheelRadius =
                              (state.gyroDelta * DriveSubsystem.DRIVE_BASE_RADIUS) / wheelDelta;

                          NumberFormat formatter = new DecimalFormat("#0.000");
                          System.out.println(
                              "********** Wheel Radius Characterization Results **********");
                          System.out.println(
                              "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                          System.out.println(
                              "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                          System.out.println(
                              "\tWheel Radius: "
                                  + formatter.format(wheelRadius)
                                  + " meters, "
                                  + formatter.format(Units.metersToInches(wheelRadius))
                                  + " inches");
                        })))
        .withName("Wheel Radius Characterization");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
