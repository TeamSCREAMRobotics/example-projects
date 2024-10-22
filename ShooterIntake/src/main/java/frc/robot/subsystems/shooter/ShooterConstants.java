package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;

public class ShooterConstants {

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Shooter";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(12), InvertedValue.Clockwise_Positive);

    CONFIGURATION.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(13), InvertedValue.Clockwise_Positive)
        };

    CONFIGURATION.slot0 =
        new ScreamPIDConstants(0.001, 0, 0)
            .getSlot0Configs(new FeedforwardConstants(0.143, 0.11, 0, 0.009));

    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 20;

    CONFIGURATION.velocityThreshold = 1.5; // 1.5 rps == 90 rpm
  }
}
