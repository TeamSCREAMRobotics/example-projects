// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import lombok.Getter;

public class RobotContainer {
  private static final Arm arm = new Arm();

  //private final CommandXboxController controller = new CommandXboxController(0);

  @Getter
  private static final RobotState robotState = new RobotState(arm);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //controller.a().whileTrue(getAutonomousCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
