package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.field.FieldConstants;
import frc.robot.game.GameState;
import org.littletonrobotics.junction.Logger;

public class ShootingCalculator {
  private PoseSupplier poseSupplier;
  private ChassisSpeedsSupplier chassisSpeedsSupplier;

  private double distance = 0.0;
  private boolean isMoving = false;
  private double leadAngle = 0.0;

  private GameState gameState;

  private static final double MOVING_THRESHOLD_MPS = 0.3;
  private static final double BASE_FLIGHT_TIME_SECONDS = 0.5;

  /** Flight time scaling factor per meter of distance */
  private static final double FLIGHT_TIME_PER_METER = 0.08;

  private double angleToTarget;

  private ShootingCalculator(
      PoseSupplier poseSupplier, ChassisSpeedsSupplier chassisSpeedsSupplier, GameState gameState) {
    this.poseSupplier = poseSupplier;
    this.gameState = gameState;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
  }

  private static ShootingCalculator instance;

  public static void init(
      PoseSupplier poseSupplier, ChassisSpeedsSupplier chassisSpeedsSupplier, GameState gameState) {
    instance = new ShootingCalculator(poseSupplier, chassisSpeedsSupplier, gameState);
  }

  public static ShootingCalculator getInstance() {
    if (instance == null) {
      throw new RuntimeException("ShootingCalculator never initialized");
    }
    return instance;
  }

  public static record TargetDelta(Angle angle, Distance distance) {}

  public TargetDelta update(Translation2d target) {
    var pose = poseSupplier.supply();
    double headingRad = pose.getRotation().getRadians();
    double cosH = Math.cos(headingRad);
    double sinH = Math.sin(headingRad);
    double shooterXOffset = 0; // SuperstructureConstants.SHOOTER_OFFSET_X.in(Meters);
    double shooterYOffset = 0; // SuperstructureConstants.SHOOTER_OFFSET_Y.in(Meters);
    double turretX = pose.getX() + shooterXOffset * cosH - shooterYOffset * sinH;
    double turretY = pose.getY() + shooterXOffset * sinH + shooterYOffset * cosH;

    double dx = target.getX() - turretX;
    double dy = target.getY() - turretY;

    distance = Math.sqrt(dx * dx + dy * dy);
    double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
    Logger.recordOutput("ShootingCalculator/FieldAngle", fieldAngle);
    // double robotHeading = pose.getRotation().getDegrees();
    // Logger.recordOutput("ShootingCalculator/RobotHeading", robotHeading);
    // angleToTarget = normalizeAngle(fieldAngle - robotHeading);
    Logger.recordOutput("ShootingCalculator/Target", target);
    Logger.recordOutput("ShootingCalculator/Distance", Meters.of(distance).in(Feet));
    // fieldAngle is straight to the target, calculate lead offset next
    var chassisSpeeds = chassisSpeedsSupplier.supply();
    double fieldVX =
        chassisSpeeds.vxMetersPerSecond * cosH - chassisSpeeds.vyMetersPerSecond * sinH;
    double fieldVY =
        chassisSpeeds.vxMetersPerSecond * sinH + chassisSpeeds.vyMetersPerSecond * cosH;
    double robotSpeed = Math.sqrt(fieldVX * fieldVX + fieldVY * fieldVY);
    isMoving = robotSpeed > MOVING_THRESHOLD_MPS;
    if (isMoving && distance > 0.5) {
      // Estimate flight time based on distance
      double flightTime = BASE_FLIGHT_TIME_SECONDS + (distance * FLIGHT_TIME_PER_METER);

      // Where will the robot be when the ball arrives? The ball inherits robot velocity,
      // but we need to lead the turret angle to compensate for the robot's lateral motion
      // relative to the target direction.

      // Project velocity onto perpendicular-to-target direction
      double targetAngleRad = Math.toRadians(fieldAngle);
      double perpVelocity =
          -fieldVX * Math.sin(targetAngleRad) + fieldVY * Math.cos(targetAngleRad);

      // Lead angle = atan(perpendicular_velocity * flight_time / distance)
      // SUBTRACT because the ball inherits the robot's velocity. If the robot
      // is moving left (positive perpVelocity), the ball drifts left, so we
      // aim right (negative correction) to compensate.
      leadAngle = Math.toDegrees(Math.atan2(perpVelocity * flightTime, distance));
      // Apply lead angle (subtract: we aim opposite to the drift)
      fieldAngle -= leadAngle;
      fieldAngle = normalizeAngle(fieldAngle);
    } else {
      leadAngle = 0.0;
    }
    Logger.recordOutput("ShootingCalculator/LeadAngle", -leadAngle);
    Logger.recordOutput("ShootingCalculator/FinalAngle", fieldAngle);
    return new TargetDelta(Degrees.of(fieldAngle), Meters.of(distance));
  }

  public Translation2d getHub() {
    if (gameState.getAlliance() == Alliance.Red) {
      return FieldConstants.RED_HUB;
    } else {
      return FieldConstants.BLUE_HUB;
    }
  }

  public Translation2d getNearestTrench() {
    var pose = poseSupplier.supply();
    if (gameState.getAlliance() == Alliance.Blue) {
      if (pose.getMeasureY().gt(FieldConstants.FIELD_HEIGHT.div(2))) {
        return FieldConstants.BLUE_DEPOT_TRENCH;
      } else {
        return FieldConstants.BLUE_OUTPOST_TRENCH;
      }
    } else {
      if (pose.getMeasureY().gt(FieldConstants.FIELD_HEIGHT.div(2))) {
        return FieldConstants.RED_OUTPOST_TRENCH;
      } else {
        return FieldConstants.RED_DEPOT_TRENCH;
      }
    }
  }

  private double normalizeAngle(double deg) {
    return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  }

  @FunctionalInterface
  public static interface PoseSupplier {
    public Pose2d supply();
  }

  @FunctionalInterface
  public static interface ChassisSpeedsSupplier {
    public ChassisSpeeds supply();
  }
}
