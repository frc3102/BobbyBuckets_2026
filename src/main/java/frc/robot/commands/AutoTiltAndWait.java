package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.tilt.IntakeTilt;

public class AutoTiltAndWait extends Command {
  private IntakeTilt tilt;
  private double position;

  public AutoTiltAndWait(IntakeTilt tilt, double position) {
    this.tilt = tilt;
    this.position = position;
    addRequirements(tilt);
  }

  @Override
  public boolean isFinished() {
    return tilt.inPosition(position);
  }

  @Override
  public void initialize() {
    tilt.setPosition(position);
  }
}
