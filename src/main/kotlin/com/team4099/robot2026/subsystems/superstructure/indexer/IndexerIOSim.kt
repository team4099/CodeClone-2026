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
    private val indexerSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(2),
            IndexerConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
            1.0 / IndexerConstants.GEAR_RATIO,
        ),
        DCMotor.getKrakenX60(2)
    )

    private val indexerPIDController =
        PIDController(
            IndexerConstants.PID.SIM_KP,
            IndexerConstants.PID.SIM_KI,
            IndexerConstants.PID.SIM_KD
        )
    private var indexerFFController =
        SimpleMotorFeedforward(
            IndexerConstants.PID.SIM_KS,
            IndexerConstants.PID.SIM_KV,
            IndexerConstants.PID.SIM_KA
        )
    private var appliedVoltage = 0.0.volts

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        indexerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
        inputs.floorIndexerVelocity = indexerSim.angularVelocityRadPerSec.radians.perSecond
        inputs.floorIndexerAcceleration = indexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
        inputs.floorIndexerAppliedVoltage = appliedVoltage
        inputs.floorIndexerSupplyCurrent = 0.0.amps
        inputs.floorIndexerStatorCurrent = indexerSim.currentDrawAmps.amps
        inputs.floorIndexerTorqueCurrent = 0.0.amps
        inputs.floorIndexerTemperature = 0.0.celsius

        inputs.feedWheelsIndexerVelocity = indexerSim.angularVelocityRadPerSec.radians.perSecond
        inputs.feedWheelsIndexerAcceleration = indexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
        inputs.feedWheelsIndexerAppliedVoltage = appliedVoltage
        inputs.feedWheelsIndexerSupplyCurrent = 0.0.amps
        inputs.feedWheelsIndexerStatorCurrent = indexerSim.currentDrawAmps.amps
        inputs.feedWheelsIndexerTorqueCurrent = 0.0.amps
        inputs.feedWheelsIndexerTemperature = 0.0.celsius

        inputs.sideRollerIndexerVelocity = indexerSim.angularVelocityRadPerSec.radians.perSecond
        inputs.sideRollerIndexerAcceleration = indexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
        inputs.sideRollerIndexerAppliedVoltage = appliedVoltage
        inputs.sideRollerIndexerSupplyCurrent = 0.0.amps
        inputs.sideRollerIndexerStatorCurrent = indexerSim.currentDrawAmps.amps
        inputs.sideRollerIndexerTorqueCurrent = 0.0.amps
        inputs.sideRollerIndexerTemperature = 0.0.celsius

        inputs.topBeltIndexerVelocity = indexerSim.angularVelocityRadPerSec.radians.perSecond
        inputs.topBeltIndexerAcceleration = indexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
        inputs.topBeltIndexerAppliedVoltage = appliedVoltage
        inputs.topBeltIndexerSupplyCurrent = 0.0.amps
        inputs.topBeltIndexerStatorCurrent = indexerSim.currentDrawAmps.amps
        inputs.topBeltIndexerTorqueCurrent = 0.0.amps
        inputs.topBeltIndexerTemperature = 0.0.celsius

        inputs.bottomBeltIndexerVelocity = indexerSim.angularVelocityRadPerSec.radians.perSecond
        inputs.bottomBeltIndexerAcceleration = indexerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
        inputs.bottomBeltIndexerAppliedVoltage = appliedVoltage
        inputs.bottomBeltIndexerSupplyCurrent = 0.0.amps
        inputs.bottomBeltIndexerStatorCurrent = indexerSim.currentDrawAmps.amps
        inputs.bottomBeltIndexerTorqueCurrent = 0.0.amps
        inputs.bottomBeltIndexerTemperature = 0.0.celsius
    }

    override fun setVoltage(voltage: ElectricalPotential) {
        val clampedVoltage = clamp(voltage, -IndexerConstants.VOLTAGE_COMPENSATION, IndexerConstants.VOLTAGE_COMPENSATION)
        indexerSim.setInputVoltage(clampedVoltage.inVolts)
        appliedVoltage = clampedVoltage
    }

    override fun setVelocity(velocity: AngularVelocity) {
        val pidOutput = indexerPIDController.calculate(indexerSim.angularVelocityRadPerSec.radians.perSecond, velocity)
        val ffOutput = indexerFFController.calculateWithVelocities(
            indexerSim.angularVelocityRadPerSec.radians.perSecond, velocity)
        setVoltage(pidOutput+ffOutput)
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