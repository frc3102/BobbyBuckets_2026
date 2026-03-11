package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ShootingCalculator;

public class AutoAimAtHub extends ParallelRaceGroup {

  public AutoAimAtHub(Drive drive) {
    addRequirements(drive);
    var aimCommand =
        DriveCommands.joystickDriveAtAngle(drive, () -> 0, () -> 0, AutoAimAtHub::getAngleToHub);
    var inPositionCommand =
        Commands.run(
            () -> {
              var angle = getAngleToHub();
              drive.getRotation().getMeasure().isNear(angle.getMeasure(), 0.05);
            });
    addCommands(aimCommand, inPositionCommand);
  }

  private static Rotation2d getAngleToHub() {
    var calc = ShootingCalculator.getInstance();
    var target = calc.getHub();
    var delta = calc.update(target);
    return new Rotation2d(delta.angle());
  }
}
