package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elbow.Elbow;
import frc.robot.subsystems.elbow.ElbowConstants;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderConstants;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;
import java.util.Set;

public class Arm {
  public final Shoulder shoulder = new Shoulder(ShoulderConstants.CONFIGURATION);
  public final Elbow elbow = new Elbow(ElbowConstants.CONFIGURATION);
  public final Wrist wrist = new Wrist(WristConstants.CONFIGURATION);

  public final double length1 = ShoulderConstants.LENGTH.getMeters();
  public final double length2 = ElbowConstants.LENGTH.getMeters();
  public final double length3 = WristConstants.LENGTH.getMeters();

  public Command inverse(Translation2d target) {
    double[] angles = new double[3];
    double targetX = target.getX();
    double targetY = target.getY();

    double distance = Math.hypot(targetX, targetY);

    if (distance > (length1 + length2 + length3)) {
      throw new IllegalArgumentException("Target is out of reach.");
    }

    angles[0] = Math.atan2(targetY, targetX);

    double x1 = length1 * Math.cos(angles[0]);
    double y1 = length1 * Math.sin(angles[0]);
    double dx = targetX - x1;
    double dy = targetY - y1;

    double distanceToSecondJoint = Math.sqrt(dx * dx + dy * dy);

    double angle2 =
        Math.acos(
            (length2 * length2 + distanceToSecondJoint * distanceToSecondJoint - length3 * length3)
                / (2 * length2 * distanceToSecondJoint));
    angles[1] = Math.atan2(dy, dx) - angle2;

    double angle3 =
        Math.acos(
            (length1 * length1 + length2 * length2 - distanceToSecondJoint * distanceToSecondJoint)
                / (2 * length1 * length2));
    angles[2] = Math.PI - angle3;

    return Commands.defer(
        () ->
            Commands.parallel(
                shoulder.run(
                    () -> shoulder.setSetpointPosition(Units.radiansToRotations(angles[0]))),
                elbow.run(() -> elbow.setSetpointPosition(Units.radiansToRotations(angles[1]))),
                wrist.run(() -> wrist.setSetpointPosition(Units.radiansToRotations(angles[2])))),
        Set.of(shoulder, elbow, wrist));
  }
}
