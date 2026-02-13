package frc.robot.subsystems.intake.feed;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeFeedIO {

  @AutoLog
  public static class IntakeFeedIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double positionRad = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IntakeFeedIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
