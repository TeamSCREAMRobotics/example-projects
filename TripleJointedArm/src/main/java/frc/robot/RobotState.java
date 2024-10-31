package frc.robot;

import dashboard.Ligament;
import dashboard.Mechanism;
import dashboard.MechanismVisualizer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.elbow.Elbow;
import frc.robot.subsystems.elbow.ElbowConstants;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderConstants;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

public class RobotState {
    private final Shoulder shoulder;
    private final Elbow elbow;
    private final Wrist wrist;

    private final Ligament shoulderLig;
    private final Ligament elbowLig;
    private final Ligament wristLig;

    private final Mechanism arm;

    private final MechanismVisualizer visualizer;

    public RobotState(Arm arm){
        this.shoulder = arm.shoulder;
        this.elbow = arm.elbow;
        this.wrist = arm.wrist;

        shoulderLig = new Ligament().withDynamicAngle(() -> shoulder.getAngle(), () -> Rotation2d.fromRotations(shoulder.getSetpoint())).withStaticLength(ShoulderConstants.LENGTH);
        elbowLig = new Ligament().withDynamicAngle(() -> elbow.getAngle(), () -> Rotation2d.fromRotations(elbow.getSetpoint())).withStaticLength(ElbowConstants.LENGTH);
        wristLig = new Ligament().withDynamicAngle(() -> wrist.getAngle(), () -> Rotation2d.fromRotations(wrist.getSetpoint())).withStaticLength(WristConstants.LENGTH);

        this.arm = new Mechanism("Arm", shoulderLig, elbowLig, wristLig);

        visualizer = new MechanismVisualizer(Constants.MEASURED_MECHANISM, Constants.SETPOINT_MECHANISM, this::telemetrizeMechanisms, this.arm);
    }

    private void telemetrizeMechanisms(Mechanism2d measured, Mechanism2d setpoint){
        SmartDashboard.putData(measured);
        SmartDashboard.putData(setpoint);
    }
}
