package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface SuperstructureIO {
  @AutoLog
  public static class SuperstructureIOInputs {
    public boolean conveyorConnected = false;
    public boolean kickerConnected = false;
    public boolean shooterLeaderConnected = false;
    public boolean shooterFollowerConnected = false;

    public Voltage conveyorVolts = Volts.of(0);
    public Voltage kickerVolts = Volts.of(0);
    public Voltage shooterLeaderVolts = Volts.of(0);
    public Voltage shooterFollowerVolts = Volts.of(0);

    public Current conveyorAmps = Amps.of(0);
    public Current kickerAmps = Amps.of(0);
    public Current shooterLeaderAmps = Amps.of(0);
    public Current shooterFollowerAmps = Amps.of(0);

    public AngularVelocity conveyorVelocity = RotationsPerSecond.of(0);
    public AngularVelocity kickerVelocity = RotationsPerSecond.of(0);
    public AngularVelocity shooterVelocity = RotationsPerSecond.of(0);
    public AngularVelocity shooterTargetVelocity = RotationsPerSecond.of(0);

    public Temperature conveyorTemp = Celsius.of(0);
    public Temperature kickerTemp = Celsius.of(0);
    public Temperature shooterLeaderTemp = Celsius.of(0);
    public Temperature shooterFollowerTemp = Celsius.of(0);
  }

  public default void updateInputs(SuperstructureIOInputs inputs) {}

  public default void setConveyorRPS(AngularVelocity velocity) {}

  public default void setKickerRPS(AngularVelocity velocity) {}

  public default void setShooterRPS(AngularVelocity velocity) {}

  public default boolean isAtSpeed(AngularVelocity velocity) {
    return true;
  }

  public default void stopKicker() {}

  public default void stopShooter() {}
}
