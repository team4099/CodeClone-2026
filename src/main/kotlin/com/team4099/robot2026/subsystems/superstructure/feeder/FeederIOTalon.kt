package com.team4099.robot2026.subsystems.superstructure.feeder

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.FeederConstants
import edu.wpi.first.units.measure.AngularAcceleration as WPIAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPIAngularVelocity
import edu.wpi.first.units.measure.Current as WPICurrent
import edu.wpi.first.units.measure.Temperature as WPITemp
import edu.wpi.first.units.measure.Voltage as WPIVoltage
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object FeederIOTalon : FeederIO {

  private val feederTalon: TalonFX = TalonFX(Constants.Feeder.FEEDER_MOTOR_ID)

  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  private val slot0Configs: Slot0Configs = configs.Slot0

  // create sensors
  private val feederSensor =
    ctreAngularMechanismSensor(
      feederTalon, FeederConstants.GEAR_RATIO, FeederConstants.VOLTAGE_COMPENSATION)

  // status signals for each motor
  private var feederStatorCurrentSignal: StatusSignal<WPICurrent>
  private var feederSupplyCurrentSignal: StatusSignal<WPICurrent>
  private var feederTorqueCurrentSignal: StatusSignal<WPICurrent>
  private var feederTempSignal: StatusSignal<WPITemp>
  private var feederVoltageSignal: StatusSignal<WPIVoltage>
  private var feederAccelSignal: StatusSignal<WPIAngularAcceleration>
  private var feederVelocitySignal: StatusSignal<WPIAngularVelocity>

  val voltageOut = VoltageOut(0.volts.inVolts)

  init {
    feederTalon.clearStickyFaults()

    // current limits and backup modes
    configs.CurrentLimits.SupplyCurrentLimit = FeederConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimit = FeederConstants.STATOR_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    // applying configs
    feederTalon.configurator.apply(configs)

    // define signals for each motor
    feederSupplyCurrentSignal = feederTalon.supplyCurrent
    feederStatorCurrentSignal = feederTalon.statorCurrent
    feederTorqueCurrentSignal = feederTalon.torqueCurrent
    feederVelocitySignal = feederTalon.velocity
    feederTempSignal = feederTalon.deviceTemp
    feederVoltageSignal = feederTalon.motorVoltage
    feederAccelSignal = feederTalon.acceleration
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
      feederSupplyCurrentSignal,
      feederStatorCurrentSignal,
      feederTorqueCurrentSignal,
      feederVelocitySignal,
      feederTempSignal,
      feederVoltageSignal,
      feederAccelSignal)
  }

  override fun updateInputs(inputs: FeederIO.FeederInputs) {
    updateSignals()

    inputs.feederVelocity = feederSensor.velocity
    inputs.feederAcceleration =
      (feederAccelSignal.valueAsDouble / FeederConstants.GEAR_RATIO)
        .rotations
        .perSecond
        .perSecond
    inputs.feederTemperature = feederTempSignal.valueAsDouble.celsius
    inputs.feederSupplyCurrent = feederSupplyCurrentSignal.valueAsDouble.amps
    inputs.feederStatorCurrent = feederStatorCurrentSignal.valueAsDouble.amps
    inputs.feederTorqueCurrent = feederTorqueCurrentSignal.valueAsDouble.amps
    inputs.feederAppliedVoltage = feederVoltageSignal.valueAsDouble.volts
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(
        voltage,
        lowerBound = -FeederConstants.VOLTAGE_COMPENSATION,
        upperBound = FeederConstants.VOLTAGE_COMPENSATION)
    feederTalon.setControl(voltageOut.withOutput(clampedVoltage.inVolts))
  }
}