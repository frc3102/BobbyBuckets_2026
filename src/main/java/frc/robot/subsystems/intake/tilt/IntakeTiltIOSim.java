package frc.robot.subsystems.intake.tilt;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

public class IntakeTiltIOSim implements IntakeTiltIO {

  private Angle angle = Radians.of(0);

  @Override
  public void setAngle(Angle angle) {
    this.angle = angle;
  }

  @Override
  public void updateInputs(IntakeTiltIOInputs inputs) {
    inputs.connected = true;
    inputs.position = this.angle;
  }
}
