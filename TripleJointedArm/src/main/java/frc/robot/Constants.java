package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public class Constants {

  public static final double MECH_WIDTH = Units.inchesToMeters(25);
  public static final double MECH_LENGTH = MECH_WIDTH;

  public static final Mechanism2d MEASURED_MECHANISM = new Mechanism2d(MECH_WIDTH, MECH_LENGTH);
  public static final Mechanism2d SETPOINT_MECHANISM = new Mechanism2d(MECH_WIDTH, MECH_LENGTH);
}
