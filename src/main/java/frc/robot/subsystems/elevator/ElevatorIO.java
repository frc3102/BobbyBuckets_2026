package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean connected = false;

    public Voltage appliedVolts = Volts.of(0);
    public Current supplyCurrentAmps = Amps.of(0);
    public Current statorCurrentAmps = Amps.of(0);
    public Temperature temp = Celsius.of(0);
    public AngularVelocity velocity = RotationsPerSecond.of(0);
    public Angle closedLoopReference = Radians.of(0);
    public Angle closedLoopError = Radians.of(0);
    public Distance linearPosition = Inches.of(0);
    public Angle angularPosition = Rotations.of(0);
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPosition(Distance height) {}

  public default void zeroElevator() {}

  public default void setVoltage(Voltage volts) {}
}
