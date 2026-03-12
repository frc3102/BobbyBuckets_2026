package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.superstructure.Superstructure;

public class TuningShot extends Command {
  private final LoggedTunableNumber rps = new LoggedTunableNumber("ShooterTuning/RPS", 50);
  private Superstructure superstructure;
  private AngularVelocity targetVelocity;
  private boolean atSpeed = false;

  public TuningShot(Superstructure superstructure) {
    addRequirements(superstructure);
    this.superstructure = superstructure;
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
    targetVelocity = RotationsPerSecond.of(rps.get());
    superstructure.startShooter(targetVelocity);
  }
}
