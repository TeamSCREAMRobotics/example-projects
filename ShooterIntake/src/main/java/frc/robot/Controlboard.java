package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Controlboard {
  public static final CommandXboxController controller = new CommandXboxController(0);

  public static final double STICK_DEADBAND = 0.05;

  public static boolean fieldCentric = true;

  static {
    controller.start().onTrue(Commands.runOnce(() -> fieldCentric = !fieldCentric));
  }

  public static double applyPower(double value, int power) {
    return Math.pow(value, power) * (power % 2 == 0 ? Math.signum(value) : 1);
  }

  public static Supplier<Translation2d> getRawTranslation() {
    return () -> new Translation2d(controller.getLeftY(), controller.getLeftX());
  }

  public static Supplier<Translation2d> getTranslation() {
    return () ->
        new Translation2d(
                applyPower(-MathUtil.applyDeadband(controller.getLeftY(), STICK_DEADBAND), 2),
                applyPower(-MathUtil.applyDeadband(controller.getLeftX(), STICK_DEADBAND), 2))
            .times(DrivetrainConstants.MAX_SPEED);
  }

  public static DoubleSupplier getRotation() {
    return () ->
        applyPower(-MathUtil.applyDeadband(controller.getRightX(), STICK_DEADBAND), 3)
            * DrivetrainConstants.MAX_ANGULAR_SPEED_RADS;
  }

  public static BooleanSupplier getFieldCentric() {
    return () -> fieldCentric;
  }
}
