package frc.robot.subsystems.intake;

import dev.doglog.DogLog;
import drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;

public class IntakeDeploy extends TalonFXSubsystem {

  public IntakeDeploy(TalonFXSubsystemConfiguration config) {
    super(config, DeployGoal.IDLE);
  }

  public enum DeployGoal implements TalonFXSubsystemGoal {
    IDLE(Rotation2d.fromDegrees(90), ControlType.MOTION_MAGIC_POSITION),
    DEPLOY(Rotation2d.fromDegrees(-30), ControlType.MOTION_MAGIC_POSITION);

    public final double targetRotations;
    public final ControlType controlType;

    private DeployGoal(Rotation2d angle, ControlType controlType) {
      this.targetRotations = angle.getRotations();
      this.controlType = controlType;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }

    @Override
    public DoubleSupplier target() {
      return () -> targetRotations;
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    DogLog.log(logPrefix + "Angle", getAngle());
  }
}
