package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics.", Alert.AlertType.kError);

  private static final LoggedTunableNumber coastWaitTime =
      new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
  private static final LoggedTunableNumber coastMetersPerSecondThreshold =
      new LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", .05);

  private final Timer lastMovementTimer = new Timer();

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());

  @AutoLogOutput(key = "Drive/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  @AutoLogOutput(key = "Drive/IsTipping")
  private boolean isTipping = false;

  private ChassisSpeeds antiTipSpeeds = new ChassisSpeeds();
  private static final LoggedTunableNumber tipKp =
      new LoggedTunableNumber("Drive/AntiTip/TipKp", 0.03);
  private static final LoggedTunableNumber tipThreshold =
      new LoggedTunableNumber("Drive/AntiTip/TipThreshold", 3.0);

  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint currentSetpoint;

  public static final DriveTrainSimulationConfig driveTrainSimulationConfig =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              COTS.ofMark4i(
                  DCMotor.getKrakenX60Foc(1),
                  DCMotor.getKrakenX60Foc(1),
                  COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                  2)) // L2 Gear ratio
          .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
          .withBumperSize(Inches.of(37), Inches.of(37))
          .withRobotMass(Kilogram.of(ROBOT_MASS_KG));

  private final Consumer<Pose2d> resetSimulationPoseCallback;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallback) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, "FrontLeft", TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, "FrontRight", TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, "BackLeft", TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, "BackRight", TunerConstants.BackRight);
    this.resetSimulationPoseCallback = resetSimulationPoseCallback;

    lastMovementTimer.start();
    setBrakeMode(true);

    // Usage reporting for swerve template
    HAL.report(
        FRCNetComm.tResourceType.kResourceType_RobotDrive,
        FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.setCANFD(TunerConstants.kCANBus.isNetworkFD());
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> runVelocity(speeds, feedforwards),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () ->
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) ->
            Logger.recordOutput("PathPlanner/ActivePath", activePath.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> Logger.recordOutput("PathPlanner/TargetPose", targetPose));

    setpointGenerator = new SwerveSetpointGenerator(PP_CONFIG, 10 * Math.PI);
    currentSetpoint =
        new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  public enum CoastRequest {
    AUTOMATIC,
    ALWAYS_BRAKE,
    ALWAYS_COAST
  }

  @AutoLogOutput(key = "Drive/CoastRequest")
  private CoastRequest coastRequest = CoastRequest.AUTOMATIC;

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    LoggedTracer.record("Drive/Inputs");

    // Call periodic on modules
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Drive/States/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/States/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Log 3D robot pose
    Logger.recordOutput(
        "Drive/EstimatedPosition3d",
        new Pose3d(getPose())
            .exp(
                new Twist3d(
                    0.0,
                    0.0,
                    Math.abs(gyroInputs.pitchPosition.getRadians())
                        * driveTrainSimulationConfig.trackLengthX().in(Meters)
                        / 2.0,
                    0.0,
                    gyroInputs.pitchPosition.getRadians(),
                    0.0))
            .exp(
                new Twist3d(
                    0.0,
                    0.0,
                    Math.abs(gyroInputs.rollPosition.getRadians())
                        * driveTrainSimulationConfig.trackLengthX().in(Meters)
                        / 2.0,
                    gyroInputs.rollPosition.getRadians(),
                    0.0,
                    0.0)));

    // Calculate anti-tip control
    isTipping =
        Math.abs(gyroInputs.pitchPosition.getDegrees()) > tipThreshold.get()
            || Math.abs(gyroInputs.rollPosition.getDegrees()) > tipThreshold.get();

    // Tilt direction
    Rotation2d tiltDirection =
        new Rotation2d(
            Math.atan2(
                -gyroInputs.rollPosition.getDegrees(), -gyroInputs.pitchPosition.getDegrees()));

    double correctionSpeed =
        tipKp.get()
            * -Math.hypot(
                gyroInputs.pitchPosition.getDegrees(), gyroInputs.rollPosition.getDegrees());
    correctionSpeed =
        MathUtil.clamp(
            correctionSpeed,
            -TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 2.0,
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 2.0);

    // Correction vector (field-relative)
    Translation2d correctionVector =
        new Translation2d(0, 1).rotateBy(tiltDirection).times(correctionSpeed);

    antiTipSpeeds = new ChassisSpeeds(correctionVector.getY(), correctionVector.getX(), 0);

    // Update brake mode
    // Reset movement timer if velocity above threshold
    if (Arrays.stream(modules)
        .anyMatch(
            (module) ->
                Math.abs(module.getVelocityMetersPerSec()) > coastMetersPerSecondThreshold.get())) {
      lastMovementTimer.reset();
    }

    if (DriverStation.isEnabled() && DriverStation.isFMSAttached()) {
      coastRequest = CoastRequest.ALWAYS_BRAKE;
    }

    switch (coastRequest) {
      case AUTOMATIC -> {
        if (DriverStation.isEnabled()) {
          setBrakeMode(true);
        } else if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
          setBrakeMode(false);
        }
      }
      case ALWAYS_BRAKE -> setBrakeMode(true);
      case ALWAYS_COAST -> setBrakeMode(false);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected);
    LoggedTracer.record("Drive/Periodic");
  }

  /** Set brake mode to {@code enabled} doesn't change brake mode if already set. */
  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled != enabled) {
      Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
    }
    brakeModeEnabled = enabled;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Override input if we're tipping
    if (isTipping) speeds = antiTipSpeeds;

    // Calculate module setpoints
    currentSetpoint = setpointGenerator.generateSetpoint(currentSetpoint, speeds, 0.02);
    SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Drive/States/Setpoints", setpointStates);
    Logger.recordOutput("Drive/ChassisSpeeds/Setpoints", currentSetpoint.robotRelativeSpeeds());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drive/States/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drive at the desired velocity with setpoint module forces.
   *
   * @param speeds Speeds in meters/sec
   * @param feedforwards The forces applied to each module
   */
  public void runVelocity(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    // Override input if we're tipping
    if (isTipping) speeds = antiTipSpeeds;

    // Calculate module setpoints
    SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(speeds);
    currentSetpoint = setpointGenerator.generateSetpoint(currentSetpoint, speeds, 0.02);
    SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Drive/States/SetpointsUnoptimized", setpointStatesUnoptimized);
    Logger.recordOutput("Drive/States/Setpoints", setpointStates);
    Logger.recordOutput("Drive/ChassisSpeeds/Setpoints", currentSetpoint.robotRelativeSpeeds());

    // Save module forces to swerve states for logging
    SwerveModuleState[] wheelForces = new SwerveModuleState[4];
    // Send setpoints to modules
    SwerveModuleState[] moduleStates = getModuleStates();
    for (int i = 0; i < 4; i++) {
      // Optimize state
      Rotation2d wheelAngle = moduleStates[i].angle;
      setpointStates[i].optimize(wheelAngle);
      setpointStates[i].cosineScale(wheelAngle);

      // Calculate wheel torque in direction
      var wheelForce = feedforwards.linearForces()[i];
      double wheelTorqueNm = wheelForce.in(Newtons) * TunerConstants.kWheelRadius.in(Meters);
      modules[i].runSetpoint(setpointStates[i], wheelTorqueNm);

      // Log optimized setpoints (runSetpoint mutates each state)
      Logger.recordOutput("Drive/States/SetpointsOptimized", setpointStates);

      // Save to array for logging
      wheelForces[i] = new SwerveModuleState(wheelTorqueNm, setpointStates[i].angle);
    }
    Logger.recordOutput("Drive/States/ModuleForces", wheelForces);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "Drive/States/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "Drive/ChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Drive/EstimatedPosition")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    resetSimulationPoseCallback.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
