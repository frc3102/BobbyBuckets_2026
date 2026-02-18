package frc.robot.subsystems.loader;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface LoaderIO {

  @AutoLog
  public static class LoaderIOInputs {
    public boolean connected = false;
    public AngularVelocity velocity = RadiansPerSecond.of(0);
    public Voltage appliedVolts = Volts.of(0);
    public Current currentAmps = Amps.of(0);
    public Temperature temp = Celsius.of(0);
  }

  public default void updateInputs(LoaderIOInputs inputs) {}

  public default void setVoltage(Voltage volts) {}
}
