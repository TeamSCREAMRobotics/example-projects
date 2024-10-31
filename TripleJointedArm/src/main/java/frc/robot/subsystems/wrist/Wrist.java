package frc.robot.subsystems.wrist;

import drivers.TalonFXSubsystem;
import drivers.TalonFXSubsystem.ControlType;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemGoal;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;

public class Wrist extends TalonFXSubsystem {

  public Wrist(TalonFXSubsystemConfiguration config) {
    super(config, WristGoal.HOME);
  }

  public enum WristGoal implements TalonFXSubsystemGoal {
    HOME(Rotation2d.fromDegrees(90), ControlType.MOTION_MAGIC_POSITION);

    public final Rotation2d angle;
    public final ControlType controlType;

    private WristGoal(Rotation2d targetAngle, ControlType controlType) {
      this.angle = targetAngle;
      this.controlType = controlType;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }

    @Override
    public DoubleSupplier target() {
      return () -> angle.getRotations();
    }
  }
}
