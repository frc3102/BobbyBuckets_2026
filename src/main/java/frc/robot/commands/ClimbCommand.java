package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ClimbCommand extends Command {
  private Elevator elevator;
  private double position;

  public ClimbCommand(Elevator elevator, double position) {
    addRequirements(elevator);
    this.position = position;
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    elevator.goToPosition(position);
  }

  @Override
  public boolean isFinished() {
    boolean near = elevator.getPosition().isNear(Rotations.of(position), 0.05);
    return near;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(Volts.of(0));
  }
}
