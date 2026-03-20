package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ClimbCommand extends Command {
  private Elevator elevator;
  private double position;
  private Timer timer;
  private boolean quietPeriod = false;

  public ClimbCommand(Elevator elevator, double position) {
    addRequirements(elevator);
    this.position = position;
    this.elevator = elevator;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    elevator.goToPosition(position);
  }

  @Override
  public boolean isFinished() {
    boolean done = false;
    if (quietPeriod) {
      if (timer.hasElapsed(0.25)) {
        done = true;
      }
    } else {
      boolean near = elevator.getPosition().isNear(Rotations.of(position), 0.05);
      if (near) {
        quietPeriod = true;
        timer.reset();
      }
    }

    return done;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(Volts.of(0));
    quietPeriod = false;
  }
}
