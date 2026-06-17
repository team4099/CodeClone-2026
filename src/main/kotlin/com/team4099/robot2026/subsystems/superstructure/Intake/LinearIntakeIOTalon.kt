import IntakeRollersIOTalon.motorVoltage
import IntakeRollersIOTalon.temperatureSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.units.measure.Angle as WPIAngle
import edu.wpi.first.units.measure.AngularVelocity as WPIAngularVelocity
import edu.wpi.first.units.measure.Current as WPICurrent
import edu.wpi.first.units.measure.Temperature as WPITemperature
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerMeter
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeterSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond

object LinearIntakeIOTalon : LinearIntakeIO {
  private val LintakeTalon: TalonFX = TalonFX(Constants.Lintake.LINTAKE_MOTOR_ID) // Create Motor
  private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.degrees.inDegrees)
  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  private val LintakeSensor =
      ctreLinearMechanismSensor(
          LintakeTalon,
          IntakeConstants.LinearIntakeConstants.GEAR_RATIO,
          IntakeConstants.LinearIntakeConstants.DIAMETER,
          IntakeConstants.LinearIntakeConstants.VOLTAGE_COMPENSATION)

  var statorCurrentSignal: StatusSignal<WPICurrent>
  var supplyCurrentSignal: StatusSignal<WPICurrent>
  var tempSignal: StatusSignal<WPITemperature>

  var rotorPositionSignal: StatusSignal<WPIAngle>
  var rotorVelocitySignal: StatusSignal<WPIAngularVelocity>

  init {
    LintakeTalon.clearStickyFaults()

    configs.CurrentLimits.SupplyCurrentLimit =
        IntakeConstants.LinearIntakeConstants.CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimit =
        IntakeConstants.LinearIntakeConstants.CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake

    configs.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.LinearIntakeConstants.MAX_VELOCITY.inInchesPerSecond
    configs.MotionMagic.MotionMagicAcceleration =
        IntakeConstants.LinearIntakeConstants.MAX_ACCELERATION.inInchesPerSecondPerSecond

    LintakeTalon.configurator.apply(configs)
    rotorPositionSignal = LintakeTalon.position
    rotorVelocitySignal = LintakeTalon.velocity
    statorCurrentSignal = LintakeTalon.statorCurrent
    supplyCurrentSignal = LintakeTalon.supplyCurrent
    tempSignal = LintakeTalon.deviceTemp

    zeroEncoder()
  }

  override fun configPID(
      kP: ProportionalGain<Meter, Volt>,
      kI: IntegralGain<Meter, Volt>,
      kD: DerivativeGain<Meter, Volt>
  ) {
    configs.Slot0.kP = kP.inVoltsPerMeter
    configs.Slot0.kI = kI.inVoltsPerMeterSeconds
    configs.Slot0.kD = kD.inVoltsPerMeterPerSecond

    LintakeTalon.configurator.apply(configs)
  }

  override fun configFF(
      kG: ElectricalPotential,
      kS: StaticFeedforward<Volt>,
  ) {
    configs.Slot0.kG = kG.inVolts
    configs.Slot0.kS = kS.inVolts

    LintakeTalon.configurator.apply(configs)
  }

  override fun zeroEncoder() {
    LintakeTalon.setPosition(0.0)
  }

  override fun setVoltage(targetVoltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(
            targetVoltage,
            -IntakeConstants.LinearIntakeConstants.VOLTAGE_COMPENSATION,
            IntakeConstants.LinearIntakeConstants.VOLTAGE_COMPENSATION)
    LintakeTalon.setControl(VoltageOut(clampedVoltage.inVolts))
  }

  override fun setPosition(position: Length) {
    LintakeTalon.setControl(
        motionMagicControl.withPosition(LintakeSensor.positionToRawUnits(position)).withSlot(0))
  }

  override fun updateInputs(inputs: LinearIntakeIO.LintakeIOInputs) {
    inputs.lintakeVelocity = LintakeSensor.velocity
    inputs.lintakeAppliedVoltage = motorVoltage.valueAsDouble.volts
    inputs.lintakeStatorCurrent = LinearIntakeIOTalon.statorCurrentSignal.valueAsDouble.amps
    inputs.lintakeSupplyCurrent = LinearIntakeIOTalon.supplyCurrentSignal.valueAsDouble.amps
    inputs.lintakeTemperature = temperatureSignal.valueAsDouble.celsius
  }
}
