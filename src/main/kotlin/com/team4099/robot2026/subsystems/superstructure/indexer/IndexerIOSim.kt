package com.team4099.robot2026.subsystems.superstructure.indexer

import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.IndexerConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object IndexerIOSim : IndexerIO {
  private val floorTopIndexerSim =
      FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(1),
              IndexerConstants.FloorConstants.TOP_MOMENT_OF_INERTIA.inKilogramsMeterSquared,
              1.0 / IndexerConstants.FloorConstants.TOP_GEAR_RATIO,
          ),
          DCMotor.getKrakenX60Foc(1))
  private val floorBottomIndexerSim =
    FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60Foc(1),
        IndexerConstants.FloorConstants.BOTTOM_MOMENT_OF_INERTIA.inKilogramsMeterSquared,
        1.0 / IndexerConstants.FloorConstants.BOTTOM_GEAR_RATIO,
      ),
      DCMotor.getKrakenX60Foc(1))

  private val sideRollerIndexerSim =
    FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX44Foc(1),
        IndexerConstants.SideRollerConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
        1.0 / IndexerConstants.SideRollerConstants.GEAR_RATIO,
      ),
      DCMotor.getKrakenX44Foc(1))

  private val beltTopIndexerSim =
    FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX44Foc(1),
        IndexerConstants.BeltConstants.TOP_MOMENT_OF_INERTIA.inKilogramsMeterSquared,
        1.0 / IndexerConstants.BeltConstants.GEAR_RATIO,
      ),
      DCMotor.getKrakenX44Foc(1))
  private val beltBottomIndexerSim =
    FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX44Foc(1),
        IndexerConstants.BeltConstants.BOTTOM_MOMENT_OF_INERTIA.inKilogramsMeterSquared,
        1.0 / IndexerConstants.BeltConstants.GEAR_RATIO,
      ),
      DCMotor.getKrakenX44Foc(1))

  private var floorAppliedVoltage = 0.0.volts
  private var sideRollerAppliedVoltage = 0.0.volts
  private var beltAppliedVoltage = 0.0.volts

  override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
    floorTopIndexerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    floorBottomIndexerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    sideRollerIndexerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    beltTopIndexerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    beltBottomIndexerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.floorTopIndexerVelocity = floorTopIndexerSim.angularVelocityRadPerSec.radians.perSecond
    inputs.floorBottomIndexerVelocity = floorBottomIndexerSim.angularVelocityRadPerSec.radians.perSecond
    inputs.floorTopIndexerAcceleration =
      floorTopIndexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.floorBottomIndexerAcceleration =
      floorBottomIndexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.floorIndexerAppliedVoltage = floorAppliedVoltage
    inputs.floorIndexerSupplyCurrent = 0.0.amps
    inputs.floorIndexerStatorCurrent = floorTopIndexerSim.currentDrawAmps.amps
    inputs.floorIndexerTemperature = 0.0.celsius

    inputs.sideRollerIndexerVelocity = sideRollerIndexerSim.angularVelocityRadPerSec.radians.perSecond
    inputs.sideRollerIndexerAcceleration =
      sideRollerIndexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.sideRollerIndexerAppliedVoltage = sideRollerAppliedVoltage
    inputs.sideRollerIndexerSupplyCurrent = 0.0.amps
    inputs.sideRollerIndexerStatorCurrent = sideRollerIndexerSim.currentDrawAmps.amps
    inputs.sideRollerIndexerTemperature = 0.0.celsius

    inputs.topBeltIndexerVelocity = beltTopIndexerSim.angularVelocityRadPerSec.radians.perSecond
    inputs.topBeltIndexerAcceleration =
      beltTopIndexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.topBeltIndexerAppliedVoltage = beltAppliedVoltage
    inputs.topBeltIndexerSupplyCurrent = 0.0.amps
    inputs.topBeltIndexerStatorCurrent = beltTopIndexerSim.currentDrawAmps.amps
    inputs.topBeltIndexerTemperature = 0.0.celsius

    inputs.bottomBeltIndexerVelocity = beltBottomIndexerSim.angularVelocityRadPerSec.radians.perSecond
    inputs.bottomBeltIndexerAcceleration =
      beltBottomIndexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.bottomBeltIndexerAppliedVoltage = beltAppliedVoltage
    inputs.bottomBeltIndexerSupplyCurrent = 0.0.amps
    inputs.bottomBeltIndexerStatorCurrent = beltBottomIndexerSim.currentDrawAmps.amps
    inputs.bottomBeltIndexerTemperature = 0.0.celsius
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val floorClampedVoltage =
        clamp(
            voltage, -IndexerConstants.FloorConstants.VOLTAGE_COMPENSATION,
          IndexerConstants.FloorConstants.VOLTAGE_COMPENSATION)
    val sideRollerClampedVoltage =
      clamp(
        voltage, -IndexerConstants.SideRollerConstants.VOLTAGE_COMPENSATION,
        IndexerConstants.SideRollerConstants.VOLTAGE_COMPENSATION)
    val beltClampedVoltage =
      clamp(
        voltage, -IndexerConstants.BeltConstants.VOLTAGE_COMPENSATION,
        IndexerConstants.BeltConstants.VOLTAGE_COMPENSATION)
    floorTopIndexerSim.setInputVoltage(floorClampedVoltage.inVolts)
    floorBottomIndexerSim.setInputVoltage(floorClampedVoltage.inVolts)
    sideRollerIndexerSim.setInputVoltage(sideRollerClampedVoltage.inVolts)
    beltTopIndexerSim.setInputVoltage(beltClampedVoltage.inVolts)
    beltBottomIndexerSim.setInputVoltage(beltClampedVoltage.inVolts)

    floorAppliedVoltage = floorClampedVoltage
    sideRollerAppliedVoltage = sideRollerClampedVoltage
    beltAppliedVoltage = beltClampedVoltage
  }
}
