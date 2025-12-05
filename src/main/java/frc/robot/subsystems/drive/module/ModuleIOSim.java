package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. Voltage control based. */
public class ModuleIOSim implements ModuleIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private static final double DRIVE_KS = 0.03;
  private static final double DRIVE_KV_ROT =
      0.91035; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);

  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController steerMotor;

  private boolean driveClosedLoop = false;
  private boolean steerClosedLoop = false;
  private final PIDController driveController;
  private final PIDController steerController;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double steerAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor =
        moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(80.0));
    this.steerMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(60));

    this.driveController = new PIDController(0.05, 0.0, 0.0);
    this.steerController = new PIDController(8.0, 0.0, 0.0);

    // Enable wrapping for steer PID
    steerController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(
                  moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }
    if (steerClosedLoop) {
      steerAppliedVolts =
          steerController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
    } else {
      steerController.reset();
    }

    // Update simulation state
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    steerMotor.requestVoltage(Volts.of(steerAppliedVolts));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));
    inputs.driveTemperatureCelsius = (double) 15715 / 900;

    // Update steer inputs
    inputs.steerConnected = true;
    inputs.steerEncoderConnected = true;
    inputs.steerAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.steerPosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.steerVelocityRadPerSec =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.steerAppliedVolts = steerAppliedVolts;
    inputs.steerCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));
    inputs.steerTemperatureCelsius = (double) 15715 / 900;

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometrySteerPositions = moduleSimulation.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setSteerOpenLoop(double output) {
    steerClosedLoop = false;
    steerAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double feedforward) {
    driveClosedLoop = true;
    driveFFVolts = feedforward;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setSteerPosition(Rotation2d rotation) {
    steerClosedLoop = true;
    steerController.setSetpoint(rotation.getRadians());
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveController.setPID(kP, kI, kD);
  }

  @Override
  public void setSteerPID(double kP, double kI, double kD) {
    steerController.setPID(kP, kI, kD);
  }
}
