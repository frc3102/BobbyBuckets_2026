package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.tilt.IntakeTilt;

public class JiggleHopper extends Command {
  private IntakeTilt tilt;
  private double outPosition;
  private double inPosition;

  public JiggleHopper(IntakeTilt tilt, double outPosition, double inPosition) {
    this.tilt = tilt;
    this.inPosition = inPosition;
    this.outPosition = outPosition;
    addRequirements(tilt);
  }

  @Override
  public void end(boolean interrupted) {
    tilt.setPosition(outPosition);
  }

  @Override
  public void initialize() {
    tilt.setPosition(inPosition);
  }
}
