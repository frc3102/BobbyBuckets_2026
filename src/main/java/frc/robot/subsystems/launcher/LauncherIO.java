package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {

  @AutoLog
  public static class LauncherIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public AngularVelocity velocity = RotationsPerSecond.of(0);
    public Voltage leaderAppliedVolts = Volts.of(0);
    public Voltage followerAppliedVolts = Volts.of(0);
    public Current leaderCurrentAmps = Amps.of(0);
    public Current followerCurrentAmps = Amps.of(0);
    public Temperature leaderTemp = Celsius.of(0);
    public Temperature followerTemp = Celsius.of(0);
    public AngularVelocity referenceVelocity = RotationsPerSecond.of(0);
    public AngularVelocity errorVelocity = RotationsPerSecond.of(0);
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public default void setTargetVelocity(AngularVelocity velocity) {}

  public default void setCurrent(Current amps) {}
}
