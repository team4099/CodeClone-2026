package com.team4099.robot2026.subsystems.superstructure.intake.rollers

import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object IntakeRollersIOSim : IntakeRollersIO {
  val rollersSim =
      FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              IntakeConstants.RollerIntakeConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
              1.0 / IntakeConstants.RollerIntakeConstants.GEAR_RATIO,
          ),
          DCMotor.getKrakenX60(1),
      )

  var appliedVoltage = 0.volts

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(
            voltage,
            -IntakeConstants.RollerIntakeConstants.VOLTAGE_COMPENSATION,
            IntakeConstants.RollerIntakeConstants.VOLTAGE_COMPENSATION)

    appliedVoltage = clampedVoltage
  }

  override fun updateInputs(inputs: IntakeRollersIO.RollerInputs) {
    rollersSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.rollerVelocity = rollersSim.angularVelocityRPM.rotations.perMinute
    inputs.rollerAppliedVoltage = appliedVoltage
    inputs.rollerStatorCurrent = rollersSim.currentDrawAmps.amps
    inputs.rollerSupplyCurrent = 0.0.amps
    inputs.rollerTemperature = 0.0.celsius
    inputs.rollerAcceleration =
        rollersSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
  }
}
