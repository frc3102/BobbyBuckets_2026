package frc.robot.subsystems.intake.feed;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeFeedIOSim implements IntakeFeedIO {
  private DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.004, IntakeFeedConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(1));
  private double appliedVolts = 0.0;

  @Override
  public void setVoltage(Voltage volts) {
    appliedVolts = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
  }

  @Override
  public void updateInputs(IntakeFeedIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);
    inputs.connected = true;
    inputs.velocity = sim.getAngularVelocity();
    inputs.currentAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.appliedVolts = Volts.of(appliedVolts);
  }
}
