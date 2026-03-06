package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;

public class ShootCommand extends Command {

  private Superstructure superstructure;
  private AngularVelocity targetVelocity;
  private boolean atSpeed = false;

  public ShootCommand(Superstructure superstructure, AngularVelocity velocity) {
    this.superstructure = superstructure;
    this.targetVelocity = velocity;
    addRequirements(superstructure);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.stopAll();
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
    superstructure.startShooter(targetVelocity);
  }
}
