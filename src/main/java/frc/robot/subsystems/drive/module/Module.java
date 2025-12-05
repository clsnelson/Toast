package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final LoggedTunableNumber drivekS =
      new LoggedTunableNumber("Drive/Module/DrivekS");
  private static final LoggedTunableNumber drivekV =
      new LoggedTunableNumber("Drive/Module/DrivekV");
  private static final LoggedTunableNumber drivekT =
      new LoggedTunableNumber(
          "Drive/Module/DrivekT",
          ModuleIOTalonFX.driveRatio / DCMotor.getKrakenX60Foc(1).KtNMPerAmp);
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("Drive/Module/DrivekP");
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("Drive/Module/DrivekD");
  private static final LoggedTunableNumber steerkP = new LoggedTunableNumber("Drive/Module/TurnkP");
  private static final LoggedTunableNumber steerkD = new LoggedTunableNumber("Drive/Module/TurnkD");

  static {
    if (Constants.currentMode == Constants.Mode.REAL) {
      // Real robot, use actual constants
      drivekS.initDefault(0.141);
      drivekV.initDefault(0.82);
      drivekP.initDefault(0.2);
      drivekD.initDefault(0.0);

      steerkP.initDefault(100);
      steerkD.initDefault(0.3);
    } else {
      // Simulating, use simulated constants
      drivekS.initDefault(0.03);
      drivekV.initDefault(1.0 / Units.rotationsToRadians(1.0 / 0.91035));
      drivekP.initDefault(0.1);
      drivekD.initDefault(0);
      steerkP.initDefault(10.0);
      steerkD.initDefault(0);
    }
  }

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final String name;

  private SimpleMotorFeedforward ffModel;

  private final Alert driveDisconnectedAlert;
  private final Alert steerDisconnectedAlert;
  private final Alert steerEncoderDisconnectedAlert;

  @Getter private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, String name) {
    this.io = io;
    this.name = name;

    ffModel = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());

    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on module " + name + ".", Alert.AlertType.kError);
    steerDisconnectedAlert =
        new Alert("Disconnected steer motor on module " + name + ".", Alert.AlertType.kError);
    steerEncoderDisconnectedAlert =
        new Alert("Disconnected steer encoder on module " + name + ".", Alert.AlertType.kError);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + name, inputs);
  }

  public void periodic() {
    // Update tunable numbers
    if (drivekS.hasChanged(hashCode()) || drivekV.hasChanged(hashCode())) {
      ffModel = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());
    }
    if (drivekP.hasChanged(hashCode()) || drivekD.hasChanged(hashCode())) {
      io.setDrivePID(drivekP.get(), 0, drivekD.get());
    }
    if (steerkP.hasChanged(hashCode()) || steerkD.hasChanged(hashCode())) {
      io.setSteerPID(steerkP.get(), 0, steerkD.get());
    }

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          inputs.odometryDrivePositionsRad[i] * DriveConstants.wheelRadius.in(Meters);
      Rotation2d angle = inputs.odometrySteerPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    steerDisconnectedAlert.set(!inputs.steerConnected);
    steerEncoderDisconnectedAlert.set(!inputs.steerEncoderConnected);
  }

  /**
   * Runs the module with the specified setpoint state and a setpoint wheel force used for
   * torque-based feedforward.
   */
  public void runSetpointWithFeedforward(SwerveModuleState state, double wheelTorqueNm) {
    if (ModuleIOTalonFX.driveClosedLoopOutput
        == SwerveModuleConstants.ClosedLoopOutputType.Voltage) {
      runSetpoint(state);
      return;
    }

    // Apply setpoints
    double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.wheelRadius.in(Meters);
    io.setDriveVelocity(
        speedRadPerSec, ffModel.calculate(speedRadPerSec) + wheelTorqueNm * drivekT.get());

    // Prevent wheel steering from messing with alignment
    if (Math.abs(state.angle.minus(getAngle()).getDegrees())
        < DriveConstants.steerDeadbandDegrees) {
      io.setSteerOpenLoop(0.0);
    } else {
      io.setSteerPosition(state.angle);
    }
  }

  /** Runs the module with the specified setpoint state. */
  public void runSetpoint(SwerveModuleState state) {
    double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.wheelRadius.in(Meters);
    io.setDriveVelocity(speedRadPerSec, ffModel.calculate(speedRadPerSec));

    // Prevent wheel steering from messing with alignment
    if (Math.abs(state.angle.minus(getAngle()).getDegrees())
        < DriveConstants.steerDeadbandDegrees) {
      io.setSteerOpenLoop(0.0);
    } else {
      io.setSteerPosition(state.angle);
    }
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setSteerPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setSteerOpenLoop(0.0);
  }

  /** Returns the current steer angle of the module. */
  public Rotation2d getAngle() {
    return inputs.steerPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.wheelRadius.in(Meters);
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.wheelRadius.in(Meters);
  }

  /** Returns the module position (steer angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (steer angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }

  /* Sets brake mode to {@code enabled} */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }
}
