package frc.robot.subsystems.intake.tilt;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeTiltIO {

  @AutoLog
  public static class IntakeTiltIOInputs {
    public boolean connected = false;
    public Angle position = Radians.of(0);
    public Voltage appliedVolts = Volts.of(0);
    public Current currentAmps = Amps.of(0);
    public Temperature temp = Celsius.of(0);
  }

  public default void updateInputs(IntakeTiltIOInputs inputs) {}

  public default void setAngle(Angle angle) {}

  public default void setVoltage(Voltage volts) {}
}
