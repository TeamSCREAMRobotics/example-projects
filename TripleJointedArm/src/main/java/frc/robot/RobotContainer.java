// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controlboard.Controlboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import java.util.Set;
import lombok.Getter;
import util.AllianceFlipUtil;

public class RobotContainer {
  public static final Arm arm = new Arm();

  public static final Drivetrain drivetrain = TunerConstants.DriveTrain;

  @Getter private static final RobotState robotState = new RobotState(arm);

  public RobotContainer() {
    SmartDashboard.putNumber("Wrist Target", 0);
    SmartDashboard.putNumber("IK X", 1);
    SmartDashboard.putNumber("IK Y", 1);
    configureDefaultCommands();
    configureBindings();

    if (Robot.isSimulation()) {
      drivetrain.resetPose(new Pose2d(new Translation2d(), AllianceFlipUtil.getFwdHeading()));
    }

    drivetrain.registerTelemetry(RobotContainer::telemeterizeDrivetrain);
  }

  private void configureBindings() {
    Controlboard.driveController
        .a()
        .toggleOnTrue(
            Commands.defer(
                    () ->
                        arm.inverse(
                            () -> SmartDashboard.getNumber("IK X", 1),
                            () -> SmartDashboard.getNumber("IK Y", 1)),
                    Set.of(arm.shoulder, arm.elbow))
                .alongWith(
                    Commands.defer(
                        () ->
                            arm.wrist.run(
                                () ->
                                    arm.wrist.setSetpointPosition(
                                        Units.degreesToRotations(
                                            SmartDashboard.getNumber("Wrist Target", 0)))),
                        Set.of(arm.wrist)))
                .alongWith(
                    drivetrain.applyRequest(
                        () ->
                            drivetrain
                                .getHelper()
                                .getFacingAngle(
                                    Controlboard.getTranslation().get(), Rotation2d.k180deg))));

    Controlboard.driveController
        .y()
        .whileTrue(arm.inverse(new Translation2d(1.081, 1.461), Rotation2d.fromDegrees(-85.0)));

    Controlboard.driveController
        .b()
        .whileTrue(arm.inverse(new Translation2d(0.877, 1.240), Rotation2d.fromDegrees(-90.0)));
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                Controlboard.getFieldCentric().getAsBoolean()
                    ? drivetrain
                        .getHelper()
                        .getHeadingCorrectedFieldCentric(
                            Controlboard.getTranslation().get(),
                            Controlboard.getRotation().getAsDouble())
                    : drivetrain
                        .getHelper()
                        .getRobotCentric(
                            Controlboard.getTranslation().get(),
                            Controlboard.getRotation().getAsDouble())));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static void telemeterizeDrivetrain(SwerveDriveState state) {
    DogLog.log(
        "TargetPoint",
        arm.getTargetPoint(
            new Translation2d(
                SmartDashboard.getNumber("IK X", 1), SmartDashboard.getNumber("IK Y", 1)),
            state.Pose));
    DogLog.log("RobotState/RobotPose", state.Pose);
    DogLog.log("RobotState/Subsystems/Drivetrain/MeasuredStates", state.ModuleStates);
    DogLog.log("RobotState/Subsystems/Drivetrain/SetpointStates", state.ModuleTargets);
  }
}
