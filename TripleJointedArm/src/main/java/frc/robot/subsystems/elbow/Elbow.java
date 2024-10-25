package frc.robot.subsystems.elbow;

import java.util.function.DoubleSupplier;

import drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;

public class Elbow extends TalonFXSubsystem{

    public Elbow(TalonFXSubsystemConfiguration config){
        super(config, ElbowGoal.HOME);
    }

    public enum ElbowGoal implements TalonFXSubsystemGoal{
        HOME(Rotation2d.fromDegrees(0), ControlType.MOTION_MAGIC_POSITION);

        public final Rotation2d angle;
        public final ControlType controlType;

        private ElbowGoal(Rotation2d targetAngle, ControlType controlType){
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
