package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.util.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turret = new TalonFX(TurretConstants.CAN_ID);
  private final StatusSignal<Angle> positionRot = turret.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = turret.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = turret.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = turret.getSupplyCurrent();
  private final StatusSignal<Temperature> currentTemp = turret.getDeviceTemp();

  private final MotionMagicVoltage mmRequest;
  private final PositionVoltage positionRequest;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final LoggedTunableNumber motorKP =
      new LoggedTunableNumber("Turret/kP", TurretConstants.Motor.KP);
  private final LoggedTunableNumber motorKI =
      new LoggedTunableNumber("Turret/kI", TurretConstants.Motor.KI);
  private final LoggedTunableNumber motorKD =
      new LoggedTunableNumber("Turret/kD", TurretConstants.Motor.KD);
  private final LoggedTunableNumber motorKS =
      new LoggedTunableNumber("Turret/kS", TurretConstants.Motor.KS);
  private final LoggedTunableNumber motorKV =
      new LoggedTunableNumber("Turret/kV", TurretConstants.Motor.KV);
  private final LoggedTunableNumber motorKA =
      new LoggedTunableNumber("Turret/kA", TurretConstants.Motor.KA);
  private final LoggedTunableNumber motorMMKV =
      new LoggedTunableNumber("Turret/mm_kV", TurretConstants.Motor.MM_KV);
  private final LoggedTunableNumber motorMMKA =
      new LoggedTunableNumber("Turret/mm_kA", TurretConstants.Motor.MM_KA);
  private final LoggedTunableNumber motorMMCruise =
      new LoggedTunableNumber("Turret/mm_cruise", TurretConstants.Motor.MM_CV);
  private final LoggedTunableNumber motorMMJerk =
      new LoggedTunableNumber("Turret/mm_jerk", TurretConstants.Motor.MM_JERK);

  private final ArmSystemSim turretSim;

  private Angle targetAngle = Degrees.of(0);

  public TurretIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = TurretConstants.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kP = motorKP.get();
    config.Slot0.kI = motorKI.get();
    config.Slot0.kD = motorKD.get();
    config.Slot0.kS = motorKS.get();
    config.Slot0.kV = motorKV.get();
    config.Slot0.kA = motorKA.get();
    config.MotionMagic.MotionMagicAcceleration = motorMMKA.get();
    config.MotionMagic.MotionMagicCruiseVelocity = motorMMCruise.get();
    config.MotionMagic.MotionMagicJerk = motorMMJerk.get();

    PhoenixUtil.tryUntilOk(5, () -> turret.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps, currentTemp);
    ParentDevice.optimizeBusUtilizationForAll(turret);
    zeroPosition();
    positionRequest = new PositionVoltage(0).withSlot(0);
    mmRequest = new MotionMagicVoltage(0).withSlot(0);
    turretSim =
        new ArmSystemSim(
            turret,
            false,
            TurretConstants.GEAR_RATIO,
            Inches.of(12).in(Meters),
            Pounds.of(5).in(Kilograms),
            TurretConstants.MIN_ANGLE.in(Radians),
            TurretConstants.MAX_ANGLE.in(Radians),
            0,
            "Turret");
  }

  @Override
  public void setAngle(Angle angle) {
    System.out.println("Requested: " + angle.in(Degrees));
    if (angle.gt(TurretConstants.MAX_ANGLE)) {
      angle = TurretConstants.MAX_ANGLE;
    } else if (angle.lt(TurretConstants.MIN_ANGLE)) {
      angle = TurretConstants.MIN_ANGLE;
    }
    targetAngle = angle;
    System.out.println("Target: " + angle.in(Degrees));
    System.out.println("Target rotations: " + angle.in(Rotations));
    System.out.println(
        "After gear ratio: " + Radians.of(angle.in(Rotations) * TurretConstants.GEAR_RATIO));
    var finalAngle = Rotations.of(angle.in(Rotations) * TurretConstants.GEAR_RATIO);
    System.out.println("Final Angle: " + finalAngle);
    // turret.setControl(positionRequest.withPosition(finalAngle));
    turret.setControl(mmRequest.withPosition(finalAngle));
  }

  @Override
  public void setVoltage(Voltage volts) {
    turret.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stepAngle(Angle step) {
    var targetPos = targetAngle.plus(step);
    this.setAngle(targetPos);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                positionRot, velocityRotPerSec, appliedVolts, currentAmps, currentTemp)
            .isOK();
    inputs.position = positionRot.getValue();
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.currentAmps = currentAmps.getValue();
    inputs.temp = currentTemp.getValue();
    inputs.target = targetAngle.copy();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.turret.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kV = motionMagic[4];
          config.Slot0.kA = motionMagic[5];
          // config.MotionMagic.MotionMagicExpo_kV = motionMagic[6];
          // config.MotionMagic.MotionMagicExpo_kA = motionMagic[7];
          config.MotionMagic.MotionMagicAcceleration = motionMagic[6];
          config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[7];
          config.MotionMagic.MotionMagicJerk = motionMagic[8];
          this.turret.getConfigurator().apply(config);
        },
        motorKP,
        motorKI,
        motorKD,
        motorKS,
        motorKV,
        motorKA,
        motorMMKA,
        motorMMCruise,
        motorMMJerk);

    if (Constants.getMode() == Constants.Mode.SIM) {
      this.turretSim.updateSim();
    }
  }

  @Override
  public void zeroPosition() {

    turret.setControl(new NeutralOut());
    turret.setPosition(0);
  }
}
