package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import java.io.File;
import java.util.Objects;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Controllers (bind both to the same port in sim for ease of control)
  private final CommandXboxController driver = new CommandXboxController(0);

  // Subsystems
  private final Drive drivetrain;

  // Elastic auto chooser
  private LoggedDashboardChooser<Command> autoChooser;

  // MapleSim configuration (start on the field)
  public static SwerveDriveSimulation swerveDriveSimulation =
      new SwerveDriveSimulation(
          DriveConstants.driveTrainSimulationConfig, new Pose2d(3, 3, Rotation2d.kZero));

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(DriveConstants.moduleConfigs[0]),
                new ModuleIOTalonFX(DriveConstants.moduleConfigs[1]),
                new ModuleIOTalonFX(DriveConstants.moduleConfigs[2]),
                new ModuleIOTalonFX(DriveConstants.moduleConfigs[3]),
                (robotPose) -> {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain =
            new Drive(
                new GyroIOSim(swerveDriveSimulation.getGyroSimulation()),
                new ModuleIOSim(swerveDriveSimulation.getModules()[0]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[1]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[2]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[3]),
                (robotPose) -> swerveDriveSimulation.setSimulationWorldPose(robotPose));
        drivetrain.setPose(swerveDriveSimulation.getSimulatedDriveTrainPose());
        break;

      default:
        // Replayed robot, disable IO implementations
        drivetrain =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {});
        break;
    }

    // Add drivetrain to MapleSim if in simulation
    if (RobotBase.isSimulation())
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

    setupControllerBindings();
    setupPathPlanner();
  }

  private void setupControllerBindings() {

    // DRIVE CONTROLLER
    drivetrain.setDefaultCommand(
        DriveCommands.fieldRelative(
            drivetrain,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    // Left bumper: Robot relative
    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.robotRelative(
                drivetrain,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));

    // A: X-brake
    driver.a().whileTrue(DriveCommands.brakeWithX(drivetrain));

    // Left trigger: Align to closest left branch
    driver
        .leftTrigger()
        .whileTrue(
            DriveCommands.alignToClosestReefBranch(
                drivetrain,
                DriveCommands.BranchSide.LEFT,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX()));

    // Right trigger: Align to closest right branch
    driver
        .rightTrigger()
        .whileTrue(
            DriveCommands.alignToClosestReefBranch(
                drivetrain,
                DriveCommands.BranchSide.RIGHT,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX()));

    // Reset gyro to 0 when start button is pressed.
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drivetrain.setPose(
                            new Pose2d(
                                drivetrain.getPose().getTranslation(),
                                Rotation2d.kZero.plus(
                                    AutoBuilder.shouldFlip() ? Rotation2d.kPi : Rotation2d.kZero))),
                    drivetrain)
                .ignoringDisable(true));

    // Feedforward characterization
    driver.x().whileTrue(DriveCommands.feedforwardCharacterization(drivetrain));

    // Wheel radius characterization
    driver.b().whileTrue(DriveCommands.wheelRadiusCharacterization(drivetrain));
  }

  private void setupPathPlanner() {
    // Auto chooser
    autoChooser = new LoggedDashboardChooser<>("Selected Auto");
    File autosFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");
    for (File file : Objects.requireNonNull(autosFolder.listFiles())) {
      if (!file.getName().endsWith(".auto") || file.getName().equals(".DS_Store")) continue;
      String name = file.getName().replace(".auto", "");
      autoChooser.addOption(name, new PathPlannerAuto(name, false));
      autoChooser.addOption(name + " (Mirrored)", new PathPlannerAuto(name, true));
    }
    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption(
        "Basic Leave",
        drivetrain
            .run(() -> DriveCommands.robotRelative(drivetrain, () -> 0.25, () -> 0, () -> 0))
            .withTimeout(1.0));
  }

  public void readyRobotForMatch() {
    if (autoChooser.get() instanceof PathPlannerAuto auto) {
      if (AutoBuilder.shouldFlip())
        drivetrain.setPose(FlippingUtil.flipFieldPose(auto.getStartingPose()));
      else drivetrain.setPose(auto.getStartingPose());
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
