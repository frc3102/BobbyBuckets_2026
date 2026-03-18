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
  private final StatusSignal<Angle> position = tilt.getPosition();
  private final StatusSignal<AngularVelocity> velocity = tilt.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = tilt.getMotorVoltage();
  private final StatusSignal<Current> supplyAmps = tilt.getSupplyCurrent();
  private final StatusSignal<Temperature> temp = tilt.getDeviceTemp();

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

  private final LoggedTunableNumber motorMM_CV =
      new LoggedTunableNumber("IntakeTilt/MM_CV", IntakeTiltConstants.Motor.MM_CV);
  private final LoggedTunableNumber motorMM_A =
      new LoggedTunableNumber("IntakeTilt/MM_A", IntakeTiltConstants.Motor.MM_A);
  private final LoggedTunableNumber motorMM_Jerk =
      new LoggedTunableNumber("IntakeTilt/kA", IntakeTiltConstants.Motor.MM_JERK);
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
    tiltConfig.MotionMagic.MotionMagicCruiseVelocity = motorMM_CV.get();
    tiltConfig.MotionMagic.MotionMagicAcceleration = motorMM_A.get();
    tiltConfig.MotionMagic.MotionMagicJerk = motorMM_Jerk.get();

    tryUntilOk(5, () -> tilt.getConfigurator().apply(tiltConfig, 0.25));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVolts,
        supplyAmps,
        temp);
    ParentDevice.optimizeBusUtilizationForAll(tilt);
    zeroPosition();
    tiltSim =
        new ArmSystemSim(
            tilt, false, IntakeTiltConstants.GEAR_RATIO, .3, 2, 0, 45, 45, "IntakeTilt");
  }

  @Override
  public void setAngle(Angle angle) {
    tilt.setControl(voltageRequest.withPosition(angle));
  }

  @Override
  public void updateInputs(IntakeTiltIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            position,
            velocity,
            appliedVolts,
            supplyAmps,
            temp);
    inputs.connected = status.isOK();
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.currentAmps = supplyAmps.getValue();
    inputs.position = position.getValue();
    inputs.temp = temp.getValue();

    if (Constants.TUNING_MODE) {
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
    }
    if (Constants.getMode() == Constants.Mode.SIM) {
      this.tiltSim.updateSim();
    }
  }

  @Override
  public void zeroPosition() {
    this.tilt.setPosition(0);
  }
}
