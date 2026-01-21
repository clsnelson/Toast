package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import lombok.Builder;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class DriveConstants {
  public static final Frequency odomFrequency = Hertz.of(250);

  public static final double trackWidthMeters = Units.inchesToMeters(24.0);
  public static final double driveBaseRadiusMeters =
      trackWidthMeters / Math.sqrt(2); // Assumes square drivetrain.

  public static final double maxLinearSpeedMetersPerSecond = 4.2;
  public static final double maxAngularSpeedRadPerSec = 4.2 / driveBaseRadiusMeters;
  public static final double steerDeadbandDegrees = 0.3;

  public static final Distance wheelRadius = Inches.of(2.0);

  // PathPlanner config constants
  public static final double ROBOT_MASS_KG = 74.088;
  public static final double ROBOT_MOI = 6.883;
  public static final double WHEEL_COF = 1.2;
  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new com.pathplanner.lib.config.ModuleConfig(
              DriveConstants.wheelRadius,
              MetersPerSecond.of(DriveConstants.maxLinearSpeedMetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1).withReduction(ModuleIOTalonFX.driveRatio),
              Amps.of(ModuleIOTalonFX.driveStatorCurrentLimitAmps),
              1),
          Drive.getModuleTranslations());

  // MapleSim config
  public static final DriveTrainSimulationConfig driveTrainSimulationConfig =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              COTS.ofMark4i(
                  DCMotor.getKrakenX60Foc(1),
                  DCMotor.getKrakenX60Foc(1),
                  COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                  2)) // L2 Gear ratio
          .withTrackLengthTrackWidth(Meters.of(trackWidthMeters), Meters.of(trackWidthMeters))
          .withBumperSize(Inches.of(37), Inches.of(37))
          .withRobotMass(Kilogram.of(ROBOT_MASS_KG));

  public static final ModuleConfig[] moduleConfigs = {
    // Front Left
    ModuleConfig.builder()
        .driveMotorId(3)
        .steerMotorId(7)
        .encoderId(7)
        .encoderOffset(Rotations.of(0.155029296875))
        .isSteerInverted(true)
        .isEncoderInverted(false)
        .build(),

    // Front Right
    ModuleConfig.builder()
        .driveMotorId(1)
        .steerMotorId(5)
        .encoderId(5)
        .encoderOffset(Rotations.of(-0.27978515625))
        .isSteerInverted(true)
        .isEncoderInverted(false)
        .build(),

    // Back Left
    ModuleConfig.builder()
        .driveMotorId(4)
        .steerMotorId(8)
        .encoderId(8)
        .encoderOffset(Rotations.of(-0.241943359375))
        .isSteerInverted(true)
        .isEncoderInverted(false)
        .build(),

    // Back Right
    ModuleConfig.builder()
        .driveMotorId(2)
        .steerMotorId(6)
        .encoderId(6)
        .encoderOffset(Rotations.of(-0.06494140625))
        .isSteerInverted(true)
        .isEncoderInverted(false)
        .build(),
  };

  public static class PigeonConstants {
    public static final int id = 9;
  }

  @Builder
  public record ModuleConfig(
      int steerMotorId,
      int driveMotorId,
      int encoderId,
      Angle encoderOffset,
      boolean isDriveInverted,
      boolean isSteerInverted,
      boolean isEncoderInverted) {}
}
