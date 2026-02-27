package frc.robot.subsystems.autoaim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretConstants;

public class Autoaim extends SubsystemBase {

  private final AutoaimIO io;
  private final PoseSupplier poseSupplier;
  private final RotationSupplier turreRotationSupplier;

  private double relativeX = 0.0;
  private double relativeY = 0.0;

  private double rawBearing = 0.0;
  private double distance = 0.0;

  public Autoaim(AutoaimIO io, PoseSupplier poseSupplier, RotationSupplier turreRotationSupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    this.turreRotationSupplier = turreRotationSupplier;
  }

  public void aimAt(Translation2d target) {
    var pose = poseSupplier.supply();

    double headingRad = pose.getRotation().getRadians();
    double cosH = Math.cos(headingRad);
    double sinH = Math.sin(headingRad);
    double turretXOffset = TurretConstants.Position.X_CENTER.in(Meters);
    double turretYOffset = TurretConstants.Position.Y_CENTER.in(Meters);
    double turretX = pose.getX() + turretXOffset * cosH - turretYOffset * sinH;
    double turretY = pose.getY() + turretXOffset * sinH + turretYOffset * cosH;

    relativeX = turretX - target.getX();
    relativeY = turretY - target.getY();

    double dx = target.getX() - turretX;
    double dy = target.getY() - turretY;

    distance = Math.sqrt(dx * dx + dy * dy);

    double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
    double robotHeading = pose.getRotation().getDegrees();

    double turretAngle = normalizeAngle(fieldAngle - robotHeading);
    rawBearing = turretAngle;
  }

  private double normalizeAngle(double deg) {
    return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  }

  @FunctionalInterface
  public static interface PoseSupplier {
    public Pose2d supply();
  }

  @FunctionalInterface
  public static interface RotationSupplier {
    public Rotation2d supply();
  }
}
