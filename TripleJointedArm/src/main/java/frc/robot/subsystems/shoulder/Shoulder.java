package frc.robot.subsystems.shoulder;

import drivers.TalonFXSubsystem;
import drivers.TalonFXSubsystem.ControlType;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemGoal;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;

public class Shoulder extends TalonFXSubsystem {

  public Shoulder(TalonFXSubsystemConfiguration config) {
    super(config, ShoulderGoal.HOME);
  }

  public enum ShoulderGoal implements TalonFXSubsystemGoal {
    HOME(Rotation2d.fromDegrees(110.304), ControlType.MOTION_MAGIC_POSITION);

    public final Rotation2d angle;
    public final ControlType controlType;

    private ShoulderGoal(Rotation2d targetAngle, ControlType controlType) {
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
