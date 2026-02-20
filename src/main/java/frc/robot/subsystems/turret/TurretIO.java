package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = false;
    public Angle position = Radians.of(0);
    public Voltage appliedVolts = Volts.of(0);
    public Current currentAmps = Amps.of(0);
    public Temperature temp = Celsius.of(0);
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setAngle(Angle angle) {}

  public default void stepAngle(Angle step) {}

  public default void setVoltage(Voltage volts) {}
}
