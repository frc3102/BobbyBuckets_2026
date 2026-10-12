package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;

public class ManualShootCommand extends Command {

  private Superstructure superstructure;
  private DoubleSupplier throttlePosition;
  private boolean isAtSpeed;

  private static final AngularVelocity SLOWEST_SPEED = RotationsPerSecond.of(35);
  private static final AngularVelocity FASTEST_SPEED = RotationsPerSecond.of(57);
  

  public ManualShootCommand(Superstructure superstructure, DoubleSupplier throttlePosition) {
    this.superstructure = superstructure;
    this.throttlePosition = throttlePosition;
    addRequirements(superstructure);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.stopAll();
    isAtSpeed = false;
  }

  @Override
  public void execute() {
    var speed = FASTEST_SPEED.minus(SLOWEST_SPEED).times((throttlePosition.getAsDouble() + 1.0) / 2.0).plus(SLOWEST_SPEED);
    superstructure.startShooter(speed);
    
    if (!isAtSpeed && superstructure.isAtSpeed(speed)) {
      isAtSpeed = true;
      superstructure.startConveyorAndKicker();
    }
  }

  @Override
  public void initialize() {
    isAtSpeed = false;
  }
  
}
