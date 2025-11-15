// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.Elastic;
import frc.robot.util.LoggedTracer;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  private Command currentAuto;

  public Robot() {
    // Set up data receivers & replay source
    Logger.recordMetadata("tuningMode", String.valueOf(Constants.tuningMode));
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Disable joystick connection is we aren't connected to a real field (or simulating)
    DriverStation.silenceJoystickConnectionWarning(
        !DriverStation.isFMSAttached() || RobotBase.isSimulation());

    // CTRE status signal logging (only enabled if USB drive is attached)
    SignalLogger.enableAutoLogging(false);
    if (SignalLogger.setPath("/media/sda1/ctre-logs/").isOK()) SignalLogger.start();
    else SignalLogger.stop();

    // Setup web server for downloading Elastic layouts
    // (https://frc-elastic.gitbook.io/docs/additional-features-and-references/remote-layout-downloading)
    WebServer.start(5800, Filesystem.getDeployDirectory().toString());

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    DataLogManager.log("Robot disabled");
  }

  @Override
  public void disabledPeriodic() {
    // Workaround: Check to see if the autoChooser has a new auto, then set the robot pose
    if (currentAuto != m_robotContainer.getAutonomousCommand()) {
      m_robotContainer.readyRobotForMatch();
      currentAuto = m_robotContainer.getAutonomousCommand();
      System.out.println("Robot pose set for auto.");
    }
  }

  @Override
  public void disabledExit() {
    DataLogManager.log("Exiting disabled mode...");
  }

  @Override
  public void autonomousInit() {
    DataLogManager.log("Autonomous period started");

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      DataLogManager.log("Selected Auto: " + m_autonomousCommand.getName());
      m_autonomousCommand.schedule();
    }

    Elastic.selectTab("Autonomous");

    if (RobotBase.isSimulation()) {
      SimulatedArena.getInstance().resetFieldForAuto();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    DataLogManager.log("Autonomous period ended");
    Elastic.selectTab("Teleop");
  }

  @Override
  public void teleopInit() {
    DataLogManager.log("Teleoperated period started");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    // Send notif to dashboard at the end of a match :)
    DataLogManager.log("Teleoperated period ended");
    if (DriverStation.isFMSAttached()) {
      Elastic.sendNotification(
          new Elastic.Notification(
              Elastic.NotificationLevel.INFO,
              "Good match!",
              DriverStation.getReplayNumber() > 1 ? "(again)" : ""));
    }
  }

  @Override
  public void testInit() {
    DataLogManager.log("Test period started");
    CommandScheduler.getInstance().cancelAll();
    Elastic.selectTab("Debug");
    SignalLogger.start();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    DataLogManager.log("Test mode disabled");
  }

  @Override
  public void simulationInit() {
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  @Override
  public void simulationPeriodic() {
    // Update simulated field (drivetrain, intake)
    SimulatedArena.getInstance().simulationPeriodic();

    // Log robot pose + game pieces
    Logger.recordOutput(
        "MapleSim/RobotPosition",
        RobotContainer.swerveDriveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "MapleSim/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "MapleSim/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
