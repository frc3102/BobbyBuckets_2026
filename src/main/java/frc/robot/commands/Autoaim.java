package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.field.FieldConstants;
import frc.robot.game.GameState;
import frc.robot.subsystems.intake.feed.IntakeFeedConstants;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.loader.Loader;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

public class Autoaim extends Command {
  private PoseSupplier poseSupplier;
  private ChassisSpeedsSupplier chassisSpeedsSupplier;
  private Turret turret;
  private Launcher launcher;

  private double distance = 0.0;
  private double rawBearing = 0.0;
  private boolean isMoving = false;
  private double leadAngle = 0.0;

  private Translation2d target;
  private Loader loader;
  private GameState gameState;

  private static final double MOVING_THRESHOLD_MPS = 0.3;
  private static final double BASE_FLIGHT_TIME_SECONDS = 0.5;

  /** Flight time scaling factor per meter of distance */
  private static final double FLIGHT_TIME_PER_METER = 0.08;

  public Autoaim(
      GameState gameState,
      PoseSupplier poseSupplier,
      Turret turret,
      Launcher launcher,
      Loader loader,
      ChassisSpeedsSupplier chassisSpeedsSupplier) {
    this.gameState = gameState;
    this.poseSupplier = poseSupplier;
    this.turret = turret;
    this.launcher = launcher;
    this.loader = loader;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    addRequirements(turret, launcher, loader);
  }

  @Override
  public void initialize() {
    var alliance = gameState.getAlliance();
    if (alliance == Alliance.Blue) {
      target = FieldConstants.BLUE_HUB;
    } else {
      target = FieldConstants.RED_HUB;
    }
  }

  private boolean aimTurret(Pose2d pose) {

    double headingRad = pose.getRotation().getRadians();
    double cosH = Math.cos(headingRad);
    double sinH = Math.sin(headingRad);
    double turretXOffset = TurretConstants.Position.X_CENTER.in(Meters);
    double turretYOffset = TurretConstants.Position.Y_CENTER.in(Meters);
    double turretX = pose.getX() + turretXOffset * cosH - turretYOffset * sinH;
    double turretY = pose.getY() + turretXOffset * sinH + turretYOffset * cosH;

    double dx = target.getX() - turretX;
    double dy = target.getY() - turretY;

    distance = Math.sqrt(dx * dx + dy * dy);

    double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
    double robotHeading = pose.getRotation().getDegrees();

    double turretAngle = normalizeAngle(fieldAngle - robotHeading);
    rawBearing = turretAngle;

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
      turretAngle -= leadAngle;
      turretAngle = normalizeAngle(turretAngle);
    } else {
      leadAngle = 0.0;
    }

    turret.aimAt(Degrees.of(turretAngle));
    if (turret.isAtExtent()) {
      return false;
    }
    return true;
  }

  @Override
  public void execute() {
    var pose = poseSupplier.supply();

    if (gameState.getAlliance() == Alliance.Blue) {
      if (!FieldConstants.BLUE_ZONE.contains(pose.getTranslation())) {
        launcher.startAtVoltage(Volts.of(0));
        loader.setVoltage(Volts.of(0));
        return;
      }
    } else {
      if (!FieldConstants.RED_ZONE.contains(pose.getTranslation())) {
        launcher.startAtVoltage(Volts.of(0));
        loader.setVoltage(Volts.of(0));
        return;
      }
    }
    boolean canShoot = aimTurret(pose);
    launcher.setShootAtDistance(Meters.of(distance));

    if (canShoot) {
      loader.setVoltage(IntakeFeedConstants.VOLTAGE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.aimAt(Degrees.of(0));
    loader.setVoltage(Volts.of(0));
    launcher.startAtVoltage(Volts.of(0));
  }

  private double normalizeAngle(double deg) {
    return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  }

  @FunctionalInterface
  public static interface PoseSupplier {
    public Pose2d supply();
  }

  @FunctionalInterface
  public static interface AngleConsumer {
    public void accept(Angle angle);
  }

  @FunctionalInterface
  public static interface ChassisSpeedsSupplier {
    public ChassisSpeeds supply();
  }
}
