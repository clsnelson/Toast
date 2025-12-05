package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveTemperatureCelsius = 0.0;

    public boolean steerConnected = false;
    public boolean steerEncoderConnected = false;
    public Rotation2d steerAbsolutePosition = Rotation2d.kZero;
    public Rotation2d steerPosition = Rotation2d.kZero;
    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerCurrentAmps = 0.0;
    public double steerTemperatureCelsius = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  default void setDriveOpenLoop(double output) {}

  /** Run the steer motor at the specified open loop value. */
  default void setSteerOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  default void setDriveVelocity(double velocityRadPerSec, double feedforward) {}

  /** Run the steer motor to the specified rotation. */
  default void setSteerPosition(Rotation2d rotation) {}

  /** Set P, I, and D gains for closed loop control on drive motor. */
  default void setDrivePID(double kP, double kI, double kD) {}

  /** Set P, I, and D gains for closed loop control on steer motor. */
  default void setSteerPID(double kP, double kI, double kD) {}

  /** Set brake mode on drive motor */
  default void setBrakeMode(boolean enabled) {}
}
