package com.team4099.robot2026.subsystems.superstructure.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.IndexerConstants
import com.team4099.robot2026.util.CustomLogger
import org.team4099.lib.units.ctreAngularMechanismSensor
import edu.wpi.first.units.measure.AngularAcceleration as WPIAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPIAngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature as WPITemp
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.base.Ampere
import org.team4099.lib.units.base.Second
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.inAmpsPerRadianPerSecond
import org.team4099.lib.units.derived.inAmpsPerRadians
import org.team4099.lib.units.derived.inAmpsPerRadiansPerSecond
import org.team4099.lib.units.derived.inAmpsPerRadiansPerSecondPerSecond
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerSecond
import org.team4099.lib.units.perSecond

object IndexerIOTalon : IndexerIO {
    // create motors
    private val floorTalon: TalonFX = TalonFX(Constants.Indexer.FLOOR_INDEXER_MOTOR_ID)
    private val feedWheelsTalon: TalonFX = TalonFX(Constants.Indexer.FEED_WHEELS_INDEXER_MOTOR_ID)
    private val sideRollerTalon: TalonFX = TalonFX(Constants.Indexer.SIDE_ROLLER_INDEXER_MOTOR_ID)
    private val topBeltTalon: TalonFX = TalonFX(Constants.Indexer.TOP_BELT_INDEXER_MOTOR_ID)
    private val bottomBeltTalon: TalonFX = TalonFX(Constants.Indexer.BOTTOM_BELT_INDEXER_MOTOR_ID)

    private val configs: TalonFXConfiguration = TalonFXConfiguration()
    private val slot0Configs: Slot0Configs = configs.Slot0

    // create sensors
    private val floorIndexerSensor = ctreAngularMechanismSensor(floorTalon, IndexerConstants.GEAR_RATIO,
        IndexerConstants.VOLTAGE_COMPENSATION)
    private val feedWheelsIndexerSensor = ctreAngularMechanismSensor(feedWheelsTalon, IndexerConstants.GEAR_RATIO,
        IndexerConstants.VOLTAGE_COMPENSATION)
    private val sideRollerIndexerSensor = ctreAngularMechanismSensor(sideRollerTalon, IndexerConstants.GEAR_RATIO,
        IndexerConstants.VOLTAGE_COMPENSATION)
    private val topBeltIndexerSensor = ctreAngularMechanismSensor(topBeltTalon, IndexerConstants.GEAR_RATIO,
        IndexerConstants.VOLTAGE_COMPENSATION)
    private val bottomBeltIndexerSensor = ctreAngularMechanismSensor(bottomBeltTalon, IndexerConstants.GEAR_RATIO,
        IndexerConstants.VOLTAGE_COMPENSATION)

    // status signals for each motor
    private var floorStatorCurrentSignal: StatusSignal<Current>
    private var floorSupplyCurrentSignal: StatusSignal<Current>
    private var floorTorqueCurrentSignal: StatusSignal<Current>
    private var floorTempSignal: StatusSignal<WPITemp>
    private var floorVoltageSignal: StatusSignal<Voltage>
    private var floorAccelSignal: StatusSignal<WPIAngularAcceleration>
    private var floorVelocitySignal: StatusSignal<WPIAngularVelocity>

    private var feedWheelsStatorCurrentSignal: StatusSignal<Current>
    private var feedWheelsSupplyCurrentSignal: StatusSignal<Current>
    private var feedWheelsTorqueCurrentSignal: StatusSignal<Current>
    private var feedWheelsTempSignal: StatusSignal<WPITemp>
    private var feedWheelsVoltageSignal: StatusSignal<Voltage>
    private var feedWheelsAccelSignal: StatusSignal<WPIAngularAcceleration>
    private var feedWheelsVelocitySignal: StatusSignal<WPIAngularVelocity>

    private var sideRollerStatorCurrentSignal: StatusSignal<Current>
    private var sideRollerSupplyCurrentSignal: StatusSignal<Current>
    private var sideRollerTorqueCurrentSignal: StatusSignal<Current>
    private var sideRollerTempSignal: StatusSignal<WPITemp>
    private var sideRollerVoltageSignal: StatusSignal<Voltage>
    private var sideRollerAccelSignal: StatusSignal<WPIAngularAcceleration>
    private var sideRollerVelocitySignal: StatusSignal<WPIAngularVelocity>

    private var topBeltStatorCurrentSignal: StatusSignal<Current>
    private var topBeltSupplyCurrentSignal: StatusSignal<Current>
    private var topBeltTorqueCurrentSignal: StatusSignal<Current>
    private var topBeltTempSignal: StatusSignal<WPITemp>
    private var topBeltVoltageSignal: StatusSignal<Voltage>
    private var topBeltAccelSignal: StatusSignal<WPIAngularAcceleration>
    private var topBeltVelocitySignal: StatusSignal<WPIAngularVelocity>

    private var bottomBeltStatorCurrentSignal: StatusSignal<Current>
    private var bottomBeltSupplyCurrentSignal: StatusSignal<Current>
    private var bottomBeltTorqueCurrentSignal: StatusSignal<Current>
    private var bottomBeltTempSignal: StatusSignal<WPITemp>
    private var bottomBeltVoltageSignal: StatusSignal<Voltage>
    private var bottomBeltAccelSignal: StatusSignal<WPIAngularAcceleration>
    private var bottomBeltVelocitySignal: StatusSignal<WPIAngularVelocity>

    val voltageOut = VoltageOut(0.volts.inVolts)
    val velocityControl = MotionMagicVelocityTorqueCurrentFOC(0.rotations.perSecond.inRotationsPerSecond)

    init {
        floorTalon.clearStickyFaults()
        feedWheelsTalon.clearStickyFaults()
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
        feedWheelsTalon.configurator.apply(configs)
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

        feedWheelsSupplyCurrentSignal = feedWheelsTalon.supplyCurrent
        feedWheelsStatorCurrentSignal = feedWheelsTalon.statorCurrent
        feedWheelsTorqueCurrentSignal = feedWheelsTalon.torqueCurrent
        feedWheelsVelocitySignal = feedWheelsTalon.velocity
        feedWheelsTempSignal = feedWheelsTalon.deviceTemp
        feedWheelsVoltageSignal = feedWheelsTalon.motorVoltage
        feedWheelsAccelSignal = feedWheelsTalon.acceleration

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
            feedWheelsSupplyCurrentSignal,
            feedWheelsStatorCurrentSignal,
            feedWheelsTorqueCurrentSignal,
            feedWheelsVelocitySignal,
            feedWheelsTempSignal,
            feedWheelsVoltageSignal,
            feedWheelsAccelSignal,
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
            bottomBeltAccelSignal
        )
    }

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        updateSignals()

        inputs.floorIndexerVelocity = floorIndexerSensor.velocity
        inputs.floorIndexerAcceleration =
            (floorAccelSignal.valueAsDouble / IndexerConstants.GEAR_RATIO)
                .rotations.perSecond.perSecond
        inputs.floorIndexerTemperature = floorTempSignal.valueAsDouble.celsius
        inputs.floorIndexerSupplyCurrent = floorSupplyCurrentSignal.valueAsDouble.amps
        inputs.floorIndexerStatorCurrent = floorStatorCurrentSignal.valueAsDouble.amps
        inputs.floorIndexerTorqueCurrent = floorTorqueCurrentSignal.valueAsDouble.amps
        inputs.floorIndexerAppliedVoltage = floorVoltageSignal.valueAsDouble.volts

        inputs.feedWheelsIndexerVelocity = feedWheelsIndexerSensor.velocity
        inputs.feedWheelsIndexerAcceleration =
            (feedWheelsAccelSignal.valueAsDouble / IndexerConstants.GEAR_RATIO)
                .rotations.perSecond.perSecond
        inputs.feedWheelsIndexerTemperature = feedWheelsTempSignal.valueAsDouble.celsius
        inputs.feedWheelsIndexerSupplyCurrent = feedWheelsSupplyCurrentSignal.valueAsDouble.amps
        inputs.feedWheelsIndexerStatorCurrent = feedWheelsStatorCurrentSignal.valueAsDouble.amps
        inputs.feedWheelsIndexerTorqueCurrent = feedWheelsTorqueCurrentSignal.valueAsDouble.amps
        inputs.feedWheelsIndexerAppliedVoltage = feedWheelsVoltageSignal.valueAsDouble.volts

        inputs.sideRollerIndexerVelocity = sideRollerIndexerSensor.velocity
        inputs.sideRollerIndexerAcceleration =
            (sideRollerAccelSignal.valueAsDouble / IndexerConstants.GEAR_RATIO)
                .rotations.perSecond.perSecond
        inputs.sideRollerIndexerTemperature = sideRollerTempSignal.valueAsDouble.celsius
        inputs.sideRollerIndexerSupplyCurrent = sideRollerSupplyCurrentSignal.valueAsDouble.amps
        inputs.sideRollerIndexerStatorCurrent = sideRollerStatorCurrentSignal.valueAsDouble.amps
        inputs.sideRollerIndexerTorqueCurrent = sideRollerTorqueCurrentSignal.valueAsDouble.amps
        inputs.sideRollerIndexerAppliedVoltage = sideRollerVoltageSignal.valueAsDouble.volts

        inputs.topBeltIndexerVelocity = topBeltIndexerSensor.velocity
        inputs.topBeltIndexerAcceleration =
            (topBeltAccelSignal.valueAsDouble / IndexerConstants.GEAR_RATIO)
                .rotations.perSecond.perSecond
        inputs.topBeltIndexerTemperature = topBeltTempSignal.valueAsDouble.celsius
        inputs.topBeltIndexerSupplyCurrent = topBeltSupplyCurrentSignal.valueAsDouble.amps
        inputs.topBeltIndexerStatorCurrent = topBeltStatorCurrentSignal.valueAsDouble.amps
        inputs.topBeltIndexerTorqueCurrent = topBeltTorqueCurrentSignal.valueAsDouble.amps
        inputs.topBeltIndexerAppliedVoltage = topBeltVoltageSignal.valueAsDouble.volts

        inputs.bottomBeltIndexerVelocity = bottomBeltIndexerSensor.velocity
        inputs.bottomBeltIndexerAcceleration =
            (bottomBeltAccelSignal.valueAsDouble / IndexerConstants.GEAR_RATIO)
                .rotations.perSecond.perSecond
        inputs.bottomBeltIndexerTemperature = bottomBeltTempSignal.valueAsDouble.celsius
        inputs.bottomBeltIndexerSupplyCurrent = bottomBeltSupplyCurrentSignal.valueAsDouble.amps
        inputs.bottomBeltIndexerStatorCurrent = bottomBeltStatorCurrentSignal.valueAsDouble.amps
        inputs.bottomBeltIndexerTorqueCurrent = bottomBeltTorqueCurrentSignal.valueAsDouble.amps
        inputs.bottomBeltIndexerAppliedVoltage = bottomBeltVoltageSignal.valueAsDouble.volts
    }

    override fun configurePIDCurrent(
        kP: ProportionalGain<Fraction<Radian, Second>, Ampere>,
        kI: IntegralGain<Fraction<Radian, Second>, Ampere>,
        kD: DerivativeGain<Fraction<Radian, Second>, Ampere>
    ) {
        slot0Configs.kP = kP.inAmpsPerRadianPerSecond
        slot0Configs.kI = kI.inAmpsPerRadians
        slot0Configs.kD = kD.inAmpsPerRadiansPerSecondPerSecond

        floorTalon.configurator.apply(slot0Configs)
        feedWheelsTalon.configurator.apply(slot0Configs)
        sideRollerTalon.configurator.apply(slot0Configs)
        topBeltTalon.configurator.apply(slot0Configs)
        bottomBeltTalon.configurator.apply(slot0Configs)
    }

    override fun configureFFCurrent(
        kS: StaticFeedforward<Ampere>,
        kV: VelocityFeedforward<Radian, Ampere>,
        kA: AccelerationFeedforward<Radian, Ampere>
    ) {
        slot0Configs.kS = kS.inAmperes
        slot0Configs.kV = kV.inAmpsPerRadiansPerSecond
        slot0Configs.kA = kA.inAmpsPerRadiansPerSecondPerSecond

        floorTalon.configurator.apply(slot0Configs)
        feedWheelsTalon.configurator.apply(slot0Configs)
        sideRollerTalon.configurator.apply(slot0Configs)
        topBeltTalon.configurator.apply(slot0Configs)
        bottomBeltTalon.configurator.apply(slot0Configs)
    }

    override fun setVoltage(voltage: ElectricalPotential) {
        val clampedVoltage = clamp(
            voltage,
            lowerBound = -IndexerConstants.VOLTAGE_COMPENSATION,
            upperBound = IndexerConstants.VOLTAGE_COMPENSATION)
        floorTalon.setControl(voltageOut.withOutput(clampedVoltage.inVolts))
        feedWheelsTalon.setControl(voltageOut.withOutput(clampedVoltage.inVolts))
        sideRollerTalon.setControl(voltageOut.withOutput(clampedVoltage.inVolts))
        topBeltTalon.setControl(voltageOut.withOutput(clampedVoltage.inVolts))
        bottomBeltTalon.setControl(voltageOut.withOutput(clampedVoltage.inVolts))
    }

    override fun setVelocity(velocity: AngularVelocity) {
        val slotUsed = 0
        CustomLogger.recordOutput("Indexer/slotUsed", slotUsed)
        floorTalon.setControl(
            velocityControl
                .withVelocity(floorIndexerSensor.velocityToRawUnits(velocity))
                .withAcceleration(floorIndexerSensor.accelerationToRawUnits(
                    IndexerConstants.MAX_ACCELERATION)))
        feedWheelsTalon.setControl(
            velocityControl
                .withVelocity(feedWheelsIndexerSensor.velocityToRawUnits(velocity))
                .withAcceleration(feedWheelsIndexerSensor.accelerationToRawUnits(
                    IndexerConstants.MAX_ACCELERATION)))
        sideRollerTalon.setControl(
            velocityControl
                .withVelocity(sideRollerIndexerSensor.velocityToRawUnits(velocity))
                .withAcceleration(sideRollerIndexerSensor.accelerationToRawUnits(
                    IndexerConstants.MAX_ACCELERATION)))
        topBeltTalon.setControl(
            velocityControl
                .withVelocity(topBeltIndexerSensor.velocityToRawUnits(velocity))
                .withAcceleration(topBeltIndexerSensor.accelerationToRawUnits(
                    IndexerConstants.MAX_ACCELERATION)))
        bottomBeltTalon.setControl(
            velocityControl
                .withVelocity(bottomBeltIndexerSensor.velocityToRawUnits(velocity))
                .withAcceleration(bottomBeltIndexerSensor.accelerationToRawUnits(
                    IndexerConstants.MAX_ACCELERATION)))
    }
}