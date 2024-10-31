package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.signals.InvertedValue;
import data.Length;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import pid.ScreamPIDConstants;
import sim.SimWrapper;

public class ShoulderConstants {

  public static final double REDUCTION = 34.0;

  public static final Length LENGTH = Length.fromInches(36);

  public static final SingleJointedArmSim SIM =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          REDUCTION,
          0.01,
          LENGTH.getMeters(),
          -Math.toRadians(180),
          Math.toRadians(180),
          false,
          Math.toRadians(90));

  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(10.0, 0, 0);

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Shoulder";
    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;

    CONFIGURATION.simConstants =
        new TalonFXSubsystemSimConstants(new SimWrapper(SIM), SIM_GAINS.getPIDController());

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(0), InvertedValue.Clockwise_Positive);

    CONFIGURATION.sensorToMechRatio = REDUCTION;
  }
}
