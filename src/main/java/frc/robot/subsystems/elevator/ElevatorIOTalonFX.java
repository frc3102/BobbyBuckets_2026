package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team6328.util.LoggedTunableNumber;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX elevator = new TalonFX(ElevatorConstants.CAN_ID);
  private final StatusSignal<Voltage> appliedVolts = elevator.getMotorVoltage();
  private final StatusSignal<Current> statorCurrent = elevator.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrent = elevator.getSupplyCurrent();
  private final StatusSignal<Angle> position = elevator.getPosition();
  private final StatusSignal<AngularVelocity> velocity = elevator.getVelocity();
  private final StatusSignal<Temperature> temp = elevator.getDeviceTemp();

  private final LoggedTunableNumber motorKP =
      new LoggedTunableNumber("Elevator/kP", ElevatorConstants.Motor.KP);
  private final LoggedTunableNumber motorKI =
      new LoggedTunableNumber("Elevator/kI", ElevatorConstants.Motor.KI);
  private final LoggedTunableNumber motorKD =
      new LoggedTunableNumber("Elevator/kD", ElevatorConstants.Motor.KD);

  private final LoggedTunableNumber motorKS =
      new LoggedTunableNumber("Elevator/kS", ElevatorConstants.Motor.KS);
  private final LoggedTunableNumber motorKG =
      new LoggedTunableNumber("Elevator/kG", ElevatorConstants.Motor.KG);
  private final LoggedTunableNumber motorKV =
      new LoggedTunableNumber("Elevator/kV", ElevatorConstants.Motor.KV);
  private final LoggedTunableNumber motorKA =
      new LoggedTunableNumber("Elevator/kA", ElevatorConstants.Motor.KA);
  private final LoggedTunableNumber motorKVExpo =
      new LoggedTunableNumber("Elevator/kV_Expo", ElevatorConstants.Motor.KV_EXPO);
  private final LoggedTunableNumber motorKAExpo =
      new LoggedTunableNumber("Elevator/kA_Expo", ElevatorConstants.Motor.KA_EXPO);

  // private final ElevatorSystemSim elevatorSim;

  private final DynamicMotionMagicExpoVoltage positionRequest;

  public ElevatorIOTalonFX() {

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, appliedVolts, statorCurrent, supplyCurrent, position, velocity, temp);
    ParentDevice.optimizeBusUtilizationForAll(elevator);

    positionRequest = new DynamicMotionMagicExpoVoltage(0, motorKVExpo.get(), motorKAExpo.get());
  }

  @Override
  public void setPosition(Distance height) {
    // TODO Auto-generated method stub
    ElevatorIO.super.setPosition(height);
  }

  @Override
  public void setVoltage(Voltage volts) {
    // TODO Auto-generated method stub
    ElevatorIO.super.setVoltage(volts);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // TODO Auto-generated method stub
    ElevatorIO.super.updateInputs(inputs);
  }

  @Override
  public void zeroElevator() {
    // TODO Auto-generated method stub
    ElevatorIO.super.zeroElevator();
  }
}
