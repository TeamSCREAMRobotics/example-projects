package frc.robot.subsystems.drivetrain;

import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import pid.ScreamPIDConstants;

public final class DrivetrainConstants {
  public static final double MAX_SPEED = TunerConstants.SPEED_12V_MPS;
  public static final double MAX_ANGULAR_SPEED_RADS = 8.0;

  public static final int NUM_MODULES = 4;

  public static final ScreamPIDConstants HEADING_CORRECTION_CONSTANTS =
      new ScreamPIDConstants(8.0, 0.0, 0.0);
}
