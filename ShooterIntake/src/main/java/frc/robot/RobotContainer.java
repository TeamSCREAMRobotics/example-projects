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
import frc.robot.subsystems.intake.IntakeConstants.IntakeGoal;
import frc.robot.subsystems.intake.IntakeDeploy;
import frc.robot.subsystems.intake.IntakeDeploy.DeployGoal;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeRollers.RollersGoal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterGoal;
import frc.robot.subsystems.shooter.ShooterConstants;

public class RobotContainer {

  private static final Drivetrain drivetrain = TunerConstants.DriveTrain;
  private static final IntakeDeploy intakeDeploy = new IntakeDeploy(IntakeConstants.DEPLOY_CONFIG);
  private static final IntakeRollers intakeRollers =
      new IntakeRollers(IntakeConstants.DEPLOY_CONFIG);
  private static final IndexerStageOne indexerStageOne =
      new IndexerStageOne(IndexerConstants.STAGE1_CONFIG);
  private static final IndexerStageTwo indexerStageTwo =
      new IndexerStageTwo(IndexerConstants.STAGE2_CONFIG);
  private static final Shooter shooter = new Shooter(ShooterConstants.CONFIGURATION);

  private static final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {
    controller
        .rightTrigger()
        .and(controller.rightBumper().negate()) // While the right trigger is pressed and the right bumper is not pressed, execute whileTrue.
        .whileTrue(
            Commands.parallel(
                applyIntakeGoal(IntakeGoal.INTAKE), // Apply intake goal
                indexerStageOne
                    .applyGoal(IndexerGoal.INTAKE)
                    .until(
                        () -> indexerStageOne.beamTriggered() && indexerStageTwo.beamTriggered()), // Apply intake goal to stage one until both beams are triggered.
                indexerStageTwo
                    .applyGoal(IndexerGoal.INTAKE)
                    .until(() -> indexerStageTwo.beamTriggered()))); // Apply intake goal to stage two until second beam is triggered.

    controller
        .rightBumper()
        .whileTrue(shooter.applyGoal(ShooterGoal.SHOOT)) // While right bumper is pressed, apply shoot goal to shooter.
        .and(() -> shooter.atGoal())
        .whileTrue(applyIndexerGoalBoth(IndexerGoal.FEED)) // If the shooter is at the goal and right bumper is still pressed, apply feed goal to indexer.
        .and(controller.rightTrigger())
        .whileTrue(applyIntakeGoal(IntakeGoal.INTAKE)); // If the right trigger is pressed as well, apply intake goal.

    controller
        .leftBumper()
        .whileTrue( // While left bumper is pressed, apply eject goal to all.
            Commands.parallel(
                shooter.applyGoal(ShooterGoal.EJECT),
                applyIndexerGoalBoth(IndexerGoal.EJECT),
                intakeDeploy.applyGoal(DeployGoal.IDLE),
                intakeRollers.applyGoal(RollersGoal.EJECT)));
  }

  private void configureDefaultCommands() {
    // Since TalonFXSubsystem configures the default goal as the default command, we only need to do
    // it for the drivetrain.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                Controlboard.getFieldCentric().getAsBoolean()
                    ? drivetrain.helper.getFieldCentric(
                        Controlboard.getTranslation().get(),
                        Controlboard.getRotation().getAsDouble())
                    : drivetrain.helper.getRobotCentric(
                        Controlboard.getTranslation().get(),
                        Controlboard.getRotation().getAsDouble())));
  }

  // When subsystems are always used in tandem, it's simpler to combine them to a single goal.
  public Command applyIntakeGoal(IntakeGoal goal) {
    return Commands.parallel(
        intakeDeploy.applyGoal(goal.deployGoal), intakeRollers.applyGoal(goal.rollersGoal));
  }

  public Command applyIndexerGoalBoth(IndexerGoal goal) {
    return Commands.parallel(indexerStageOne.applyGoal(goal), indexerStageTwo.applyGoal(goal));
  }
}
