package com.team4099.robot2026.subsystems.superstructure.indexer

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
import com.team4099.robot2026.config.constants.IndexerConstants
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

object IndexerIOTalon : IndexerIO {

  private val floorTalon: TalonFX = TalonFX(Constants.Indexer.FLOOR_INDEXER_MOTOR_ID)
  private val sideRollerTalon: TalonFX = TalonFX(Constants.Indexer.SIDE_ROLLER_INDEXER_MOTOR_ID)
  private val topBeltTalon: TalonFX = TalonFX(Constants.Indexer.TOP_BELT_INDEXER_MOTOR_ID)
  private val bottomBeltTalon: TalonFX = TalonFX(Constants.Indexer.BOTTOM_BELT_INDEXER_MOTOR_ID)

  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  private val slot0Configs: Slot0Configs = configs.Slot0

  // create sensors
  private val floorIndexerSensor =
      ctreAngularMechanismSensor(
          floorTalon, IndexerConstants.GEAR_RATIO, IndexerConstants.VOLTAGE_COMPENSATION)
  private val sideRollerIndexerSensor =
      ctreAngularMechanismSensor(
          sideRollerTalon, IndexerConstants.GEAR_RATIO, IndexerConstants.VOLTAGE_COMPENSATION)
  private val topBeltIndexerSensor =
      ctreAngularMechanismSensor(
          topBeltTalon, IndexerConstants.GEAR_RATIO, IndexerConstants.VOLTAGE_COMPENSATION)
  private val bottomBeltIndexerSensor =
      ctreAngularMechanismSensor(
          bottomBeltTalon, IndexerConstants.GEAR_RATIO, IndexerConstants.VOLTAGE_COMPENSATION)

  // status signals for each motor
  private var floorStatorCurrentSignal: StatusSignal<WPICurrent>
  private var floorSupplyCurrentSignal: StatusSignal<WPICurrent>
  private var floorTorqueCurrentSignal: StatusSignal<WPICurrent>
  private var floorTempSignal: StatusSignal<WPITemp>
  private var floorVoltageSignal: StatusSignal<WPIVoltage>
  private var floorAccelSignal: StatusSignal<WPIAngularAcceleration>
  private var floorVelocitySignal: StatusSignal<WPIAngularVelocity>

  private var sideRollerStatorCurrentSignal: StatusSignal<WPICurrent>
  private var sideRollerSupplyCurrentSignal: StatusSignal<WPICurrent>
  private var sideRollerTorqueCurrentSignal: StatusSignal<WPICurrent>
  private var sideRollerTempSignal: StatusSignal<WPITemp>
  private var sideRollerVoltageSignal: StatusSignal<WPIVoltage>
  private var sideRollerAccelSignal: StatusSignal<WPIAngularAcceleration>
  private var sideRollerVelocitySignal: StatusSignal<WPIAngularVelocity>

  private var topBeltStatorCurrentSignal: StatusSignal<WPICurrent>
  private var topBeltSupplyCurrentSignal: StatusSignal<WPICurrent>
  private var topBeltTorqueCurrentSignal: StatusSignal<WPICurrent>
  private var topBeltTempSignal: StatusSignal<WPITemp>
  private var topBeltVoltageSignal: StatusSignal<WPIVoltage>
  private var topBeltAccelSignal: StatusSignal<WPIAngularAcceleration>
  private var topBeltVelocitySignal: StatusSignal<WPIAngularVelocity>

  private var bottomBeltStatorCurrentSignal: StatusSignal<WPICurrent>
  private var bottomBeltSupplyCurrentSignal: StatusSignal<WPICurrent>
  private var bottomBeltTorqueCurrentSignal: StatusSignal<WPICurrent>
  private var bottomBeltTempSignal: StatusSignal<WPITemp>
  private var bottomBeltVoltageSignal: StatusSignal<WPIVoltage>
  private var bottomBeltAccelSignal: StatusSignal<WPIAngularAcceleration>
  private var bottomBeltVelocitySignal: StatusSignal<WPIAngularVelocity>

  val voltageOut = VoltageOut(0.volts.inVolts)

  init {
    floorTalon.clearStickyFaults()
    sideRollerTalon.clearStickyFaults()
    topBeltTalon.clearStickyFaults()
    bottomBeltTalon.clearStickyFaults()

    // current limits and backup modes
    configs.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    // applying configs
    floorTalon.configurator.apply(configs)
    sideRollerTalon.configurator.apply(configs)
    topBeltTalon.configurator.apply(configs)
    bottomBeltTalon.configurator.apply(configs)

    // define signals for each motor
    floorSupplyCurrentSignal = floorTalon.supplyCurrent
    floorStatorCurrentSignal = floorTalon.statorCurrent
    floorTorqueCurrentSignal = floorTalon.torqueCurrent
    floorVelocitySignal = floorTalon.velocity
    floorTempSignal = floorTalon.deviceTemp
    floorVoltageSignal = floorTalon.motorVoltage
    floorAccelSignal = floorTalon.acceleration

    sideRollerSupplyCurrentSignal = sideRollerTalon.supplyCurrent
    sideRollerStatorCurrentSignal = sideRollerTalon.statorCurrent
    sideRollerTorqueCurrentSignal = sideRollerTalon.torqueCurrent
    sideRollerVelocitySignal = sideRollerTalon.velocity
    sideRollerTempSignal = sideRollerTalon.deviceTemp
    sideRollerVoltageSignal = sideRollerTalon.motorVoltage
    sideRollerAccelSignal = sideRollerTalon.acceleration

    topBeltSupplyCurrentSignal = topBeltTalon.supplyCurrent
    topBeltStatorCurrentSignal = topBeltTalon.statorCurrent
    topBeltTorqueCurrentSignal = topBeltTalon.torqueCurrent
    topBeltVelocitySignal = topBeltTalon.velocity
    topBeltTempSignal = topBeltTalon.deviceTemp
    topBeltVoltageSignal = topBeltTalon.motorVoltage
    topBeltAccelSignal = topBeltTalon.acceleration

    bottomBeltSupplyCurrentSignal = bottomBeltTalon.supplyCurrent
    bottomBeltStatorCurrentSignal = bottomBeltTalon.statorCurrent
    bottomBeltTorqueCurrentSignal = bottomBeltTalon.torqueCurrent
    bottomBeltVelocitySignal = bottomBeltTalon.velocity
    bottomBeltTempSignal = bottomBeltTalon.deviceTemp
    bottomBeltVoltageSignal = bottomBeltTalon.motorVoltage
    bottomBeltAccelSignal = bottomBeltTalon.acceleration
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
        floorSupplyCurrentSignal,
        floorStatorCurrentSignal,
        floorTorqueCurrentSignal,
        floorVelocitySignal,
        floorTempSignal,
        floorVoltageSignal,
        floorAccelSignal,
        sideRollerSupplyCurrentSignal,
        sideRollerStatorCurrentSignal,
        sideRollerTorqueCurrentSignal,
        sideRollerVelocitySignal,
        sideRollerTempSignal,
        sideRollerVoltageSignal,
        sideRollerAccelSignal,
        topBeltSupplyCurrentSignal,
        topBeltStatorCurrentSignal,
        topBeltTorqueCurrentSignal,
        topBeltVelocitySignal,
        topBeltTempSignal,
        topBeltVoltageSignal,
        topBeltAccelSignal,
        bottomBeltSupplyCurrentSignal,
        bottomBeltStatorCurrentSignal,
        bottomBeltTorqueCurrentSignal,
        bottomBeltVelocitySignal,
        bottomBeltTempSignal,
        bottomBeltVoltageSignal,
        bottomBeltAccelSignal)
  }

  override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
    updateSignals()

    inputs.floorIndexerVelocity = floorIndexerSensor.velocity
    inputs.floorIndexerAcceleration =
        (floorAccelSignal.valueAsDouble / IndexerConstants.GEAR_RATIO).rotations.perSecond.perSecond
    inputs.floorIndexerTemperature = floorTempSignal.valueAsDouble.celsius
    inputs.floorIndexerSupplyCurrent = floorSupplyCurrentSignal.valueAsDouble.amps
    inputs.floorIndexerStatorCurrent = floorStatorCurrentSignal.valueAsDouble.amps
    inputs.floorIndexerTorqueCurrent = floorTorqueCurrentSignal.valueAsDouble.amps
    inputs.floorIndexerAppliedVoltage = floorVoltageSignal.valueAsDouble.volts

    inputs.sideRollerIndexerVelocity = sideRollerIndexerSensor.velocity
    inputs.sideRollerIndexerAcceleration =
        (sideRollerAccelSignal.valueAsDouble / IndexerConstants.GEAR_RATIO)
            .rotations
            .perSecond
            .perSecond
    inputs.sideRollerIndexerTemperature = sideRollerTempSignal.valueAsDouble.celsius
    inputs.sideRollerIndexerSupplyCurrent = sideRollerSupplyCurrentSignal.valueAsDouble.amps
    inputs.sideRollerIndexerStatorCurrent = sideRollerStatorCurrentSignal.valueAsDouble.amps
    inputs.sideRollerIndexerTorqueCurrent = sideRollerTorqueCurrentSignal.valueAsDouble.amps
    inputs.sideRollerIndexerAppliedVoltage = sideRollerVoltageSignal.valueAsDouble.volts

    inputs.topBeltIndexerVelocity = topBeltIndexerSensor.velocity
    inputs.topBeltIndexerAcceleration =
        (topBeltAccelSignal.valueAsDouble / IndexerConstants.GEAR_RATIO)
            .rotations
            .perSecond
            .perSecond
    inputs.topBeltIndexerTemperature = topBeltTempSignal.valueAsDouble.celsius
    inputs.topBeltIndexerSupplyCurrent = topBeltSupplyCurrentSignal.valueAsDouble.amps
    inputs.topBeltIndexerStatorCurrent = topBeltStatorCurrentSignal.valueAsDouble.amps
    inputs.topBeltIndexerTorqueCurrent = topBeltTorqueCurrentSignal.valueAsDouble.amps
    inputs.topBeltIndexerAppliedVoltage = topBeltVoltageSignal.valueAsDouble.volts

    inputs.bottomBeltIndexerVelocity = bottomBeltIndexerSensor.velocity
    inputs.bottomBeltIndexerAcceleration =
        (bottomBeltAccelSignal.valueAsDouble / IndexerConstants.GEAR_RATIO)
            .rotations
            .perSecond
            .perSecond
    inputs.bottomBeltIndexerTemperature = bottomBeltTempSignal.valueAsDouble.celsius
    inputs.bottomBeltIndexerSupplyCurrent = bottomBeltSupplyCurrentSignal.valueAsDouble.amps
    inputs.bottomBeltIndexerStatorCurrent = bottomBeltStatorCurrentSignal.valueAsDouble.amps
    inputs.bottomBeltIndexerTorqueCurrent = bottomBeltTorqueCurrentSignal.valueAsDouble.amps
    inputs.bottomBeltIndexerAppliedVoltage = bottomBeltVoltageSignal.valueAsDouble.volts
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(
            voltage,
            lowerBound = -IndexerConstants.VOLTAGE_COMPENSATION,
            upperBound = IndexerConstants.VOLTAGE_COMPENSATION)
    floorTalon.setControl(voltageOut.withOutput(clampedVoltage.inVolts))
    sideRollerTalon.setControl(voltageOut)
    topBeltTalon.setControl(voltageOut)
    bottomBeltTalon.setControl(voltageOut)
  }
}
