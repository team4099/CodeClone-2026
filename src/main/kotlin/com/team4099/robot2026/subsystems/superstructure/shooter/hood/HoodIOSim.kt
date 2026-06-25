package com.team4099.robot2026.subsystems.superstructure.shooter.hood

import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.HoodConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object HoodIOSim : HoodIO {
  private val hoodSim: FlywheelSim =
      FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX44(1),
              HoodConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
              1.0 / HoodConstants.GEAR_RATIO,
          ),
          DCMotor.getKrakenX44(1))

  private val hoodPIDController =
      PIDController(HoodConstants.PID.SIM_KP, HoodConstants.PID.SIM_KI, HoodConstants.PID.SIM_KD)
  private var hoodFFController =
      SimpleMotorFeedforward(
          HoodConstants.PID.SIM_KS, HoodConstants.PID.SIM_KV, HoodConstants.PID.SIM_KA)

  override fun updateInputs(inputs: HoodIO.HoodInputs) {
    hoodSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    inputs.hoodAcceleration = hoodSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.hoodVelocity = hoodSim.angularVelocityRadPerSec.radians.perSecond
    inputs.hoodPosition = 0.0.radians
    // no clue
    inputs.hoodSupplyCurrent = 0.0.amps
    inputs.hoodStatorCurrent = hoodSim.currentDrawAmps.amps
    inputs.hoodTorqueCurrent = hoodSim.currentDrawAmps.amps.absoluteValue
    inputs.hoodVoltage = hoodSim.inputVoltage.volts
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(voltage, -HoodConstants.VOLTAGE_COMPENSATION, HoodConstants.VOLTAGE_COMPENSATION)
    hoodSim.setInputVoltage(clampedVoltage.inVolts)
  }

  override fun setPosition(position: Angle) {
    HoodIO.HoodInputs.hoodPosition = position.inRadians
    // idk why this doenst work
  }

  override fun configurePIDVoltage(
      kP: ProportionalGain<Radian, Volt>,
      kI: IntegralGain<Radian, Volt>,
      kD: DerivativeGain<Radian, Volt>
  ) {
    hoodPIDController.setPID(kP, kI, kD)
    // how to do if not in type
    // kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
    // kI: IntegralGain<Fraction<Radian, Second>, Volt>,
    // kD: DerivativeGain<Fraction<Radian, Second>, Volt>
    // do i have to make it like <velocity<rad>, <volt>
  }

  override fun configureFFVoltage(
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Radian, Volt>,
      kA: AccelerationFeedforward<Radian, Volt>
  ) {
    hoodFFController = SimpleMotorFeedforward(kS, kV, kA)
  }
  // why does ff have a var but pid have a val? is it bc ff predicts while pid is static?

}
