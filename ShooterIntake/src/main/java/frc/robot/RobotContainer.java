// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class RobotContainer {

  public record Subsystems(
      Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter) {}

  private static final Intake intake = new Intake(IntakeConstants.SUBSYSTEM_CONSTANTS);
  private static final Indexer indexer =
      new Indexer(IndexerConstants.STAGE_1_CONSTANTS, IndexerConstants.STAGE_2_CONSTANTS);
  private static final Shooter shooter = new Shooter(ShooterConstants.SUBSYSTEM_CONSTANTS);

  CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
              intake.applyGoal(IntakeGoal.INTAKE), 
              indexer.applyStageOneGoal(IndexerGoal.INTAKE).until(() -> indexer.getBothStagesTriggered()),
              indexer.applyStageTwoGoal(IndexerGoal.INTAKE).until(() -> indexer.getStageTwoTriggered())));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
