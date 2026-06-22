package com.team4099.robot2026.subsystems.superstructure.feeder

import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.FeederConstants
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

object FeederIOSim : FeederIO {
  private val feederSim =
    FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60Foc(1),
        FeederConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
        1.0 / FeederConstants.GEAR_RATIO,
      ),
      DCMotor.getKrakenX60Foc(1))

  private val feederPIDController =
    PIDController(
      FeederConstants.PID.SIM_KP, FeederConstants.PID.SIM_KI, FeederConstants.PID.SIM_KD)
  private var feederFFController =
    SimpleMotorFeedforward(
      FeederConstants.PID.SIM_KS, FeederConstants.PID.SIM_KV, FeederConstants.PID.SIM_KA)
  private var appliedVoltage = 0.0.volts

  override fun updateInputs(inputs: FeederIO.FeederInputs) {
    feederSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    inputs.feederVelocity = feederSim.angularVelocityRadPerSec.radians.perSecond
    inputs.feederAcceleration =
      feederSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.feederAppliedVoltage = appliedVoltage
    inputs.feederSupplyCurrent = 0.0.amps
    inputs.feederStatorCurrent = feederSim.currentDrawAmps.amps
    inputs.feederTorqueCurrent = 0.0.amps
    inputs.feederTemperature = 0.0.celsius
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(
        voltage, -FeederConstants.VOLTAGE_COMPENSATION, FeederConstants.VOLTAGE_COMPENSATION)
    feederSim.setInputVoltage(clampedVoltage.inVolts)
    appliedVoltage = clampedVoltage
  }

  override fun setVelocity(velocity: AngularVelocity) {
    val pidOutput =
      feederPIDController.calculate(
        feederSim.angularVelocityRadPerSec.radians.perSecond, velocity)
    val ffOutput =
      feederFFController.calculateWithVelocities(
        feederSim.angularVelocityRadPerSec.radians.perSecond, velocity)
    setVoltage(pidOutput + ffOutput)
  }

  override fun configPIDVoltage(
    kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
    kI: IntegralGain<Fraction<Radian, Second>, Volt>,
    kD: DerivativeGain<Fraction<Radian, Second>, Volt>
  ) {
    feederPIDController.setPID(kP, kI, kD)
  }

  override fun configureFFVoltage(
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Radian, Volt>,
    kA: AccelerationFeedforward<Radian, Volt>
  ) {
    feederFFController = SimpleMotorFeedforward(kS, kV, kA)
  }
}