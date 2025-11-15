package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final boolean tuningMode = false;
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // AprilTag layout
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static class AutoAlignConstants {
    public static final double TRANSLATION_P = 9;
    public static final double TRANSLATION_I = 0;
    public static final double TRANSLATION_D = 0.1;

    public static final double HEADING_P = 5;
    public static final double HEADING_I = 0;
    public static final double HEADING_D = 0.4;
  }
}
