package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.ShootingCalculator;

public class AutoDistanceShoot extends Command {

  private Superstructure superstructure;
  private AngularVelocity targetVelocity;
  private boolean atSpeed = false;

  public AutoDistanceShoot(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.stopAll();
    atSpeed = false;
  }

  @Override
  public void execute() {
    if (!atSpeed) {
      if (superstructure.isAtSpeed(targetVelocity)) {
        atSpeed = true;
        superstructure.startConveyorAndKicker();
      }
    }
  }

  @Override
  public void initialize() {
    var calc = ShootingCalculator.getInstance();
    var hub = calc.getHub();
    var target = calc.update(hub);
    var speed = ShootingCalculator.rpsForDistance(target.distance());
    targetVelocity = speed;
    superstructure.startShooter(speed);
  }
}
