package com.team4099.robot2026.subsystems.superstructure.indexer

import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.IndexerConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.base.Second
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object IndexerIOSim : IndexerIO {
  private val floorIndexerSim =
      FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(1),
              IndexerConstants.FloorConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
              1.0 / IndexerConstants.FloorConstants.GEAR_RATIO,
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
  private val beltIndexerSim =
    FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX44Foc(2),
        IndexerConstants.BeltConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
        1.0 / IndexerConstants.BeltConstants.GEAR_RATIO,
      ),
      DCMotor.getKrakenX44Foc(2))

  private val indexerPIDController =
      PIDController(
          IndexerConstants.PID.SIM_KP, IndexerConstants.PID.SIM_KI, IndexerConstants.PID.SIM_KD)
  private var indexerFFController =
      SimpleMotorFeedforward(
          IndexerConstants.PID.SIM_KS, IndexerConstants.PID.SIM_KV, IndexerConstants.PID.SIM_KA)
  private var floorAppliedVoltage = 0.0.volts
  private var sideRollerAppliedVoltage = 0.0.volts
  private var beltAppliedVoltage = 0.0.volts

  override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
    floorIndexerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    sideRollerIndexerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    beltIndexerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)


    inputs.floorIndexerVelocity = floorIndexerSim.angularVelocityRadPerSec.radians.perSecond
    inputs.floorIndexerAcceleration =
      floorIndexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.floorIndexerAppliedVoltage = floorAppliedVoltage
    inputs.floorIndexerSupplyCurrent = 0.0.amps
    inputs.floorIndexerStatorCurrent = floorIndexerSim.currentDrawAmps.amps
    inputs.floorIndexerTorqueCurrent = 0.0.amps
    inputs.floorIndexerTemperature = 0.0.celsius

    inputs.sideRollerIndexerVelocity = sideRollerIndexerSim.angularVelocityRadPerSec.radians.perSecond
    inputs.sideRollerIndexerAcceleration =
      sideRollerIndexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.sideRollerIndexerAppliedVoltage = sideRollerAppliedVoltage
    inputs.sideRollerIndexerSupplyCurrent = 0.0.amps
    inputs.sideRollerIndexerStatorCurrent = sideRollerIndexerSim.currentDrawAmps.amps
    inputs.sideRollerIndexerTorqueCurrent = 0.0.amps
    inputs.sideRollerIndexerTemperature = 0.0.celsius

    inputs.topBeltIndexerVelocity = beltIndexerSim.angularVelocityRadPerSec.radians.perSecond
    inputs.topBeltIndexerAcceleration =
      beltIndexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.topBeltIndexerAppliedVoltage = beltAppliedVoltage
    inputs.topBeltIndexerSupplyCurrent = 0.0.amps
    inputs.topBeltIndexerStatorCurrent = beltIndexerSim.currentDrawAmps.amps
    inputs.topBeltIndexerTorqueCurrent = 0.0.amps
    inputs.topBeltIndexerTemperature = 0.0.celsius

    inputs.bottomBeltIndexerVelocity = beltIndexerSim.angularVelocityRadPerSec.radians.perSecond
    inputs.bottomBeltIndexerAcceleration =
      beltIndexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.bottomBeltIndexerAppliedVoltage = beltAppliedVoltage
    inputs.bottomBeltIndexerSupplyCurrent = 0.0.amps
    inputs.bottomBeltIndexerStatorCurrent = beltIndexerSim.currentDrawAmps.amps
    inputs.bottomBeltIndexerTorqueCurrent = 0.0.amps
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
    floorIndexerSim.setInputVoltage(floorClampedVoltage.inVolts)
    sideRollerIndexerSim.setInputVoltage(sideRollerClampedVoltage.inVolts)
    beltIndexerSim.setInputVoltage(beltClampedVoltage.inVolts)

    floorAppliedVoltage = floorClampedVoltage
    sideRollerAppliedVoltage = sideRollerClampedVoltage
    beltAppliedVoltage = beltClampedVoltage
  }

  override fun configPIDVoltage(
      kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
      kI: IntegralGain<Fraction<Radian, Second>, Volt>,
      kD: DerivativeGain<Fraction<Radian, Second>, Volt>
  ) {
    indexerPIDController.setPID(kP, kI, kD)
  }

  override fun configureFFVoltage(
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Radian, Volt>,
      kA: AccelerationFeedforward<Radian, Volt>
  ) {
    indexerFFController = SimpleMotorFeedforward(kS, kV, kA)
  }
}
