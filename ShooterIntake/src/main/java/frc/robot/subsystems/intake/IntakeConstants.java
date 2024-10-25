package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import frc.robot.subsystems.intake.IntakeDeploy.DeployGoal;
import frc.robot.subsystems.intake.IntakeRollers.RollersGoal;

public class IntakeConstants {

  public static final double DEPLOY_REDUCTION = 16.0;
  public static final double ROLLERS_REDUCTION = 3.0;

  public static final TalonFXSubsystemConfiguration DEPLOY_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    DEPLOY_CONFIG.name = "IntakeDeploy";

    DEPLOY_CONFIG.codeEnabled = true;
    DEPLOY_CONFIG.logTelemetry = false;

    DEPLOY_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(8), InvertedValue.Clockwise_Positive);

    DEPLOY_CONFIG.enableSupplyCurrentLimit = true;
    DEPLOY_CONFIG.supplyCurrentLimit = 20;
    DEPLOY_CONFIG.sensorToMechRatio = DEPLOY_REDUCTION;
  }

  public static final TalonFXSubsystemConfiguration ROLLERS_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    ROLLERS_CONFIG.name = "IntakeRollers";

    ROLLERS_CONFIG.codeEnabled = true;
    ROLLERS_CONFIG.logTelemetry = false;

    ROLLERS_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(9), InvertedValue.Clockwise_Positive);

    ROLLERS_CONFIG.enableSupplyCurrentLimit = true;
    ROLLERS_CONFIG.supplyCurrentLimit = 20;
    ROLLERS_CONFIG.sensorToMechRatio = ROLLERS_REDUCTION;
  }

  public enum IntakeGoal {
    IDLE(DeployGoal.IDLE, RollersGoal.IDLE),
    INTAKE(DeployGoal.DEPLOY, RollersGoal.INTAKE),
    EJECT(DeployGoal.IDLE, RollersGoal.EJECT);

    public final DeployGoal deployGoal;
    public final RollersGoal rollersGoal;

    private IntakeGoal(DeployGoal deployGoal, RollersGoal rollersGoal) {
      this.deployGoal = deployGoal;
      this.rollersGoal = rollersGoal;
    }
  }
}
