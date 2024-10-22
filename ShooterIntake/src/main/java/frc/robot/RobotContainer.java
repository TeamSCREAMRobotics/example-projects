// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerGoal;
import frc.robot.subsystems.indexer.IndexerStageOne;
import frc.robot.subsystems.indexer.IndexerStageTwo;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeDeploy;
import frc.robot.subsystems.intake.IntakeDeploy.DeployGoal;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeConstants.IntakeGoal;
import frc.robot.subsystems.intake.IntakeRollers.RollersGoal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterGoal;
import frc.robot.subsystems.shooter.ShooterConstants;
import lombok.Getter;

public class RobotContainer {

  public record Subsystems(
      Drivetrain drivetrain,
      IntakeDeploy intakeDeploy,
      IntakeRollers intakeRollers,
      IndexerStageOne indexerStageOne,
      IndexerStageTwo indexerStageTwo,
      Shooter shooter) {}

  private static final Drivetrain drivetrain = TunerConstants.DriveTrain;
  private static final IntakeDeploy intakeDeploy = new IntakeDeploy(IntakeConstants.DEPLOY_CONFIG);
  private static final IntakeRollers intakeRollers =
      new IntakeRollers(IntakeConstants.DEPLOY_CONFIG);
  private static final IndexerStageOne indexerStageOne =
      new IndexerStageOne(IndexerConstants.STAGE1_CONFIG);
  private static final IndexerStageTwo indexerStageTwo =
      new IndexerStageTwo(IndexerConstants.STAGE2_CONFIG);
  private static final Shooter shooter = new Shooter(ShooterConstants.CONFIGURATION);

  @Getter
  private static final Subsystems subsystems =
      new Subsystems(
          drivetrain, intakeDeploy, intakeRollers, indexerStageOne, indexerStageTwo, shooter);

  private static final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller
        .rightTrigger()
        .and(controller.rightBumper().negate())
        .whileTrue(
            Commands.parallel(
                applyIntakeGoal(IntakeGoal.INTAKE),
                indexerStageOne
                    .applyGoal(IndexerGoal.INTAKE)
                    .until(
                        () -> indexerStageOne.beamTriggered() && indexerStageTwo.beamTriggered()),
                indexerStageTwo
                    .applyGoal(IndexerGoal.INTAKE)
                    .until(() -> indexerStageTwo.beamTriggered())));

    controller
        .rightBumper()
        .whileTrue(shooter.applyGoal(ShooterGoal.SHOOT))
        .and(() -> shooter.atGoal())
        .whileTrue(applyIndexerGoalBoth(IndexerGoal.SHOOT))
        .and(controller.rightTrigger())
        .whileTrue(applyIntakeGoal(IntakeGoal.INTAKE));

    controller
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                shooter.applyGoal(ShooterGoal.EJECT),
                applyIndexerGoalBoth(IndexerGoal.EJECT),
                intakeDeploy.applyGoal(DeployGoal.IDLE),
                intakeRollers.applyGoal(RollersGoal.INTAKE)));
  }

  public Command applyIntakeGoal(IntakeGoal goal){
    return Commands.parallel(intakeDeploy.applyGoal(goal.deployGoal), intakeRollers.applyGoal(goal.rollersGoal));
  }

  public Command applyIndexerGoalBoth(IndexerGoal goal) {
    return Commands.parallel(indexerStageOne.applyGoal(goal), indexerStageTwo.applyGoal(goal));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
