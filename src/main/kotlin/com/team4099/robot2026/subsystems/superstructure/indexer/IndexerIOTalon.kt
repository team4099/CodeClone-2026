package com.team4099.robot2026.subsystems.superstructure.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
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

  private val floorConfig: TalonFXConfiguration = TalonFXConfiguration()
  private val sideRollerConfig: TalonFXConfiguration = TalonFXConfiguration()
  private val beltConfig: TalonFXConfiguration = TalonFXConfiguration()


  // create sensors
  private val floorTopIndexerSensor =
      ctreAngularMechanismSensor(
          floorTalon, IndexerConstants.FloorConstants.TOP_GEAR_RATIO,
        IndexerConstants.FloorConstants.VOLTAGE_COMPENSATION)
  private val floorBottomIndexerSensor =
    ctreAngularMechanismSensor(
      floorTalon, IndexerConstants.FloorConstants.BOTTOM_GEAR_RATIO,
      IndexerConstants.FloorConstants.VOLTAGE_COMPENSATION)
  private val sideRollerIndexerSensor =
      ctreAngularMechanismSensor(
          sideRollerTalon, IndexerConstants.SideRollerConstants.GEAR_RATIO,
        IndexerConstants.SideRollerConstants.VOLTAGE_COMPENSATION)
  private val topBeltIndexerSensor =
      ctreAngularMechanismSensor(
          topBeltTalon, IndexerConstants.BeltConstants.GEAR_RATIO,
        IndexerConstants.BeltConstants.VOLTAGE_COMPENSATION)
  private val bottomBeltIndexerSensor =
      ctreAngularMechanismSensor(
          bottomBeltTalon, IndexerConstants.BeltConstants.GEAR_RATIO,
        IndexerConstants.BeltConstants.VOLTAGE_COMPENSATION)

  // status signals for each motor
  private var floorStatorCurrentSignal: StatusSignal<WPICurrent>
  private var floorSupplyCurrentSignal: StatusSignal<WPICurrent>
  private var floorTempSignal: StatusSignal<WPITemp>
  private var floorVoltageSignal: StatusSignal<WPIVoltage>
  private var floorTopAccelSignal: StatusSignal<WPIAngularAcceleration>
  private var floorTopVelocitySignal: StatusSignal<WPIAngularVelocity>
  private var floorBottomAccelSignal: StatusSignal<WPIAngularAcceleration>
  private var floorBottomVelocitySignal: StatusSignal<WPIAngularVelocity>

  private var sideRollerStatorCurrentSignal: StatusSignal<WPICurrent>
  private var sideRollerSupplyCurrentSignal: StatusSignal<WPICurrent>
  private var sideRollerTempSignal: StatusSignal<WPITemp>
  private var sideRollerVoltageSignal: StatusSignal<WPIVoltage>
  private var sideRollerAccelSignal: StatusSignal<WPIAngularAcceleration>
  private var sideRollerVelocitySignal: StatusSignal<WPIAngularVelocity>

  private var topBeltStatorCurrentSignal: StatusSignal<WPICurrent>
  private var topBeltSupplyCurrentSignal: StatusSignal<WPICurrent>
  private var topBeltTempSignal: StatusSignal<WPITemp>
  private var topBeltVoltageSignal: StatusSignal<WPIVoltage>
  private var topBeltAccelSignal: StatusSignal<WPIAngularAcceleration>
  private var topBeltVelocitySignal: StatusSignal<WPIAngularVelocity>

  private var bottomBeltStatorCurrentSignal: StatusSignal<WPICurrent>
  private var bottomBeltSupplyCurrentSignal: StatusSignal<WPICurrent>
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
    floorConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.FloorConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    floorConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.FloorConstants.STATOR_CURRENT_LIMIT.inAmperes
    floorConfig.CurrentLimits.SupplyCurrentLimitEnable = true
    floorConfig.CurrentLimits.StatorCurrentLimitEnable = true
    floorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast
    floorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    sideRollerConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SideRollerConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    sideRollerConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.SideRollerConstants.STATOR_CURRENT_LIMIT.inAmperes
    sideRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true
    sideRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true
    sideRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast
    sideRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    beltConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.BeltConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    beltConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.BeltConstants.STATOR_CURRENT_LIMIT.inAmperes
    beltConfig.CurrentLimits.SupplyCurrentLimitEnable = true
    beltConfig.CurrentLimits.StatorCurrentLimitEnable = true
    beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast
    beltConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    // applying configs
    floorTalon.configurator.apply(floorConfig)
    sideRollerTalon.configurator.apply(sideRollerConfig)
    topBeltTalon.configurator.apply(beltConfig)
    bottomBeltTalon.configurator.apply(beltConfig)

    // define signals for each motor
    floorSupplyCurrentSignal = floorTalon.supplyCurrent
    floorStatorCurrentSignal = floorTalon.statorCurrent
    floorBottomVelocitySignal = floorTalon.velocity
    floorTopVelocitySignal = floorTalon.velocity
    floorTempSignal = floorTalon.deviceTemp
    floorVoltageSignal = floorTalon.motorVoltage
    floorBottomAccelSignal = floorTalon.acceleration
    floorTopAccelSignal = floorTalon.acceleration

    sideRollerSupplyCurrentSignal = sideRollerTalon.supplyCurrent
    sideRollerStatorCurrentSignal = sideRollerTalon.statorCurrent
    sideRollerVelocitySignal = sideRollerTalon.velocity
    sideRollerTempSignal = sideRollerTalon.deviceTemp
    sideRollerVoltageSignal = sideRollerTalon.motorVoltage
    sideRollerAccelSignal = sideRollerTalon.acceleration

    topBeltSupplyCurrentSignal = topBeltTalon.supplyCurrent
    topBeltStatorCurrentSignal = topBeltTalon.statorCurrent
    topBeltVelocitySignal = topBeltTalon.velocity
    topBeltTempSignal = topBeltTalon.deviceTemp
    topBeltVoltageSignal = topBeltTalon.motorVoltage
    topBeltAccelSignal = topBeltTalon.acceleration

    bottomBeltSupplyCurrentSignal = bottomBeltTalon.supplyCurrent
    bottomBeltStatorCurrentSignal = bottomBeltTalon.statorCurrent
    bottomBeltVelocitySignal = bottomBeltTalon.velocity
    bottomBeltTempSignal = bottomBeltTalon.deviceTemp
    bottomBeltVoltageSignal = bottomBeltTalon.motorVoltage
    bottomBeltAccelSignal = bottomBeltTalon.acceleration
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
        floorSupplyCurrentSignal,
        floorStatorCurrentSignal,
        floorBottomVelocitySignal,
      floorTopVelocitySignal,
        floorTempSignal,
        floorVoltageSignal,
        floorBottomAccelSignal,
      floorTopAccelSignal,
        sideRollerSupplyCurrentSignal,
        sideRollerStatorCurrentSignal,
        sideRollerVelocitySignal,
        sideRollerTempSignal,
        sideRollerVoltageSignal,
        sideRollerAccelSignal,
        topBeltSupplyCurrentSignal,
        topBeltStatorCurrentSignal,
        topBeltVelocitySignal,
        topBeltTempSignal,
        topBeltVoltageSignal,
        topBeltAccelSignal,
        bottomBeltSupplyCurrentSignal,
        bottomBeltStatorCurrentSignal,
        bottomBeltVelocitySignal,
        bottomBeltTempSignal,
        bottomBeltVoltageSignal,
        bottomBeltAccelSignal)
  }

  override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
    updateSignals()

    inputs.floorTopIndexerVelocity = floorTopIndexerSensor.velocity
    inputs.floorBottomIndexerVelocity = floorBottomIndexerSensor.velocity
    inputs.floorTopIndexerAcceleration =
        (floorTopAccelSignal.valueAsDouble * IndexerConstants.FloorConstants.TOP_GEAR_RATIO)
          .rotations.perSecond.perSecond
    inputs.floorBottomIndexerAcceleration =
      (floorBottomAccelSignal.valueAsDouble * IndexerConstants.FloorConstants.BOTTOM_GEAR_RATIO)
        .rotations.perSecond.perSecond
    inputs.floorIndexerTemperature = floorTempSignal.valueAsDouble.celsius
    inputs.floorIndexerSupplyCurrent = floorSupplyCurrentSignal.valueAsDouble.amps
    inputs.floorIndexerStatorCurrent = floorStatorCurrentSignal.valueAsDouble.amps
    inputs.floorIndexerAppliedVoltage = floorVoltageSignal.valueAsDouble.volts

    inputs.sideRollerIndexerVelocity = sideRollerIndexerSensor.velocity
    inputs.sideRollerIndexerAcceleration =
        (sideRollerAccelSignal.valueAsDouble * IndexerConstants.SideRollerConstants.GEAR_RATIO)
            .rotations.perSecond.perSecond
    inputs.sideRollerIndexerTemperature = sideRollerTempSignal.valueAsDouble.celsius
    inputs.sideRollerIndexerSupplyCurrent = sideRollerSupplyCurrentSignal.valueAsDouble.amps
    inputs.sideRollerIndexerStatorCurrent = sideRollerStatorCurrentSignal.valueAsDouble.amps
    inputs.sideRollerIndexerAppliedVoltage = sideRollerVoltageSignal.valueAsDouble.volts

    inputs.topBeltIndexerVelocity = topBeltIndexerSensor.velocity
    inputs.topBeltIndexerAcceleration =
        (topBeltAccelSignal.valueAsDouble * IndexerConstants.BeltConstants.GEAR_RATIO)
            .rotations.perSecond.perSecond
    inputs.topBeltIndexerTemperature = topBeltTempSignal.valueAsDouble.celsius
    inputs.topBeltIndexerSupplyCurrent = topBeltSupplyCurrentSignal.valueAsDouble.amps
    inputs.topBeltIndexerStatorCurrent = topBeltStatorCurrentSignal.valueAsDouble.amps
    inputs.topBeltIndexerAppliedVoltage = topBeltVoltageSignal.valueAsDouble.volts

    inputs.bottomBeltIndexerVelocity = bottomBeltIndexerSensor.velocity
    inputs.bottomBeltIndexerAcceleration =
        (bottomBeltAccelSignal.valueAsDouble * IndexerConstants.BeltConstants.GEAR_RATIO)
            .rotations.perSecond.perSecond
    inputs.bottomBeltIndexerTemperature = bottomBeltTempSignal.valueAsDouble.celsius
    inputs.bottomBeltIndexerSupplyCurrent = bottomBeltSupplyCurrentSignal.valueAsDouble.amps
    inputs.bottomBeltIndexerStatorCurrent = bottomBeltStatorCurrentSignal.valueAsDouble.amps
    inputs.bottomBeltIndexerAppliedVoltage = bottomBeltVoltageSignal.valueAsDouble.volts
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val floorClampedVoltage =
        clamp(
            voltage,
            lowerBound = -IndexerConstants.FloorConstants.VOLTAGE_COMPENSATION,
            upperBound = IndexerConstants.FloorConstants.VOLTAGE_COMPENSATION)
    val sideRollerClampedVoltage =
      clamp(
        voltage,
        lowerBound = -IndexerConstants.SideRollerConstants.VOLTAGE_COMPENSATION,
        upperBound = IndexerConstants.SideRollerConstants.VOLTAGE_COMPENSATION)
    val beltClampedVoltage =
      clamp(
        voltage,
        lowerBound = -IndexerConstants.BeltConstants.VOLTAGE_COMPENSATION,
        upperBound = IndexerConstants.BeltConstants.VOLTAGE_COMPENSATION)
    floorTalon.setControl(voltageOut.withOutput(floorClampedVoltage.inVolts))
    sideRollerTalon.setControl(voltageOut.withOutput(sideRollerClampedVoltage.inVolts))
    topBeltTalon.setControl(voltageOut.withOutput(beltClampedVoltage.inVolts))
    bottomBeltTalon.setControl(voltageOut.withOutput(beltClampedVoltage.inVolts))
  }
}
