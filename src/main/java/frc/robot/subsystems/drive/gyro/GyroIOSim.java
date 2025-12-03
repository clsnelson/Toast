package frc.robot.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/** IO implementation for a MapleSim Swerve Drive Simulation. */
public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  private static final LoggedTunableNumber simulatedGyroPitch =
      new LoggedTunableNumber("Drive/SimulatedGyroPitch");
  private static final LoggedTunableNumber simulatedGyroRoll =
      new LoggedTunableNumber("Drive/SimulatedGyroRoll");

  static {
    simulatedGyroPitch.initDefault(0.0);
    simulatedGyroRoll.initDefault(0.0);
  }

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);
    inputs.pitchPosition = Rotation2d.fromDegrees(simulatedGyroPitch.get());
    inputs.pitchVelocityRadPerSec = 0.0;
    inputs.rollPosition = Rotation2d.fromDegrees(simulatedGyroRoll.get());
    inputs.rollVelocityRadPerSec = 0.0;
    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }
}
