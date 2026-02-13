package frc.robot.subsystems.intake.feed;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeFeedSim implements IntakeFeedIO {
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getCIM(1), 0.004, IntakeFeedConstants.MOTOR_REDUCTION),
          DCMotor.getCIM(1));
  private double appliedVolts = 0.0;

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void updateInputs(IntakeFeedIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);
    inputs.connected = true;
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.appliedVolts = appliedVolts;
  }
}
