package frc.robot.subsystems.intake.tilt;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team3061.sim.ArmSystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class IntakeTiltIOTalonFX implements IntakeTiltIO {

  private final TalonFX tilt = new TalonFX(IntakeTiltConstants.CAN_ID);
  private final StatusSignal<Angle> tiltPositionRot = tilt.getPosition();
  private final StatusSignal<AngularVelocity> tiltVelocityRotPerSec = tilt.getVelocity();
  private final StatusSignal<Voltage> tiltAppliedVolts = tilt.getMotorVoltage();
  private final StatusSignal<Current> tiltCurrentAmps = tilt.getSupplyCurrent();
  private final StatusSignal<Temperature> tiltCurrentTemp = tilt.getDeviceTemp();

  private final MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

  private final LoggedTunableNumber motorKP =
      new LoggedTunableNumber("IntakeTilt/kP", IntakeTiltConstants.Motor.KP);
  private final LoggedTunableNumber motorKI =
      new LoggedTunableNumber("IntakeTilt/kI", IntakeTiltConstants.Motor.KI);
  private final LoggedTunableNumber motorKD =
      new LoggedTunableNumber("IntakeTilt/kD", IntakeTiltConstants.Motor.KD);

  private final LoggedTunableNumber motorKS =
      new LoggedTunableNumber("IntakeTilt/kS", IntakeTiltConstants.Motor.KS);
  private final LoggedTunableNumber motorKG =
      new LoggedTunableNumber("IntakeTilt/kG", IntakeTiltConstants.Motor.KG);
  private final LoggedTunableNumber motorKV =
      new LoggedTunableNumber("IntakeTilt/kV", IntakeTiltConstants.Motor.KV);
  private final LoggedTunableNumber motorKA =
      new LoggedTunableNumber("IntakeTilt/kA", IntakeTiltConstants.Motor.KA);

  private final ArmSystemSim tiltSim;

  public IntakeTiltIOTalonFX() {
    var tiltConfig = new TalonFXConfiguration();
    tiltConfig.CurrentLimits.SupplyCurrentLimit = IntakeTiltConstants.Motor.CURRENT_LIMIT;
    tiltConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tiltConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tiltConfig.Slot0.kP = motorKP.get();
    tiltConfig.Slot0.kI = motorKI.get();
    tiltConfig.Slot0.kD = motorKD.get();

    tiltConfig.Slot0.kS = motorKS.get();
    tiltConfig.Slot0.kG = motorKG.get();
    tiltConfig.Slot0.kV = motorKV.get();
    tiltConfig.Slot0.kA = motorKA.get();

    tryUntilOk(5, () -> tilt.getConfigurator().apply(tiltConfig, 0.25));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        tiltPositionRot,
        tiltVelocityRotPerSec,
        tiltAppliedVolts,
        tiltCurrentAmps,
        tiltCurrentTemp);
    ParentDevice.optimizeBusUtilizationForAll(tilt);

    tiltSim =
        new ArmSystemSim(
            tilt, false, IntakeTiltConstants.GEAR_RATIO, .3, 2, 0, 45, 45, "IntakeTilt");
  }

  @Override
  public void setAngle(Angle angle) {

    tilt.setControl(voltageRequest.withPosition(angle.times(IntakeTiltConstants.GEAR_RATIO)));
  }

  @Override
  public void updateInputs(IntakeTiltIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            tiltPositionRot,
            tiltVelocityRotPerSec,
            tiltAppliedVolts,
            tiltCurrentAmps,
            tiltCurrentTemp);
    inputs.connected = status.isOK();
    inputs.appliedVolts = tiltAppliedVolts.getValue();
    inputs.currentAmps = tiltCurrentAmps.getValue();
    inputs.position = tiltPositionRot.getValue().div(IntakeTiltConstants.GEAR_RATIO);
    inputs.temp = tiltCurrentTemp.getValue();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.tilt.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kG = motionMagic[4];
          config.Slot0.kA = motionMagic[5];
          config.Slot0.kV = motionMagic[6];
          this.tilt.getConfigurator().apply(config);
        },
        motorKP,
        motorKI,
        motorKD,
        motorKS,
        motorKG,
        motorKA,
        motorKV);
    if (Constants.getMode() == Constants.Mode.SIM) {
      this.tiltSim.updateSim();
    }
  }
}
