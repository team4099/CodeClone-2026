package com.team4099.robot2026.subsystems.superstructure.shooter.flywheel

import com.ctre.phoenix6.SignalLogger
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.FieldConstants
import com.team4099.robot2026.config.constants.ShooterConstants
import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import com.team4099.robot2026.util.Velocity2d
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat.N1
import edu.wpi.first.math.Nat.N2
import edu.wpi.first.math.Vector
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage as WPILibVoltage
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import java.util.function.Consumer
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sqrt
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.inRotationsPerSecond
import org.team4099.lib.units.max
import org.team4099.lib.units.min
import org.team4099.lib.units.perSecond

class Flywheel(private val io: FlywheelIO) : ControlledByStateMachine() {
  val inputs = FlywheelIO.FlywheelInputs()
  var flywheelVoltageTarget: ElectricalPotential = 0.0.volts
    private set

  var flywheelVelocityTarget: AngularVelocity = 0.0.degrees.perSecond
    private set

  val isAtTargetedVelocity: Boolean
    get() =
        (currentRequest is Request.FlywheelRequest.TargetVelocity &&
            (inputs.flywheelLeaderVelocity - flywheelVelocityTarget).absoluteValue <
                ShooterConstants.SHOOTER_TOLERANCE)

  var currentState: FlywheelState = FlywheelState.UNINITIALIZED
  var currentRequest: Request.FlywheelRequest = Request.FlywheelRequest.Idle()
    set(value) {
      when (value) {
        is Request.FlywheelRequest.OpenLoop -> {
          flywheelVoltageTarget = value.flywheelVoltage
        }
        is Request.FlywheelRequest.TargetVelocity -> {
          flywheelVelocityTarget = value.targetVelocity
        }
        else -> {}
      }
      field = value
    }

  val flywheelTestVel =
      LoggedTunableValue(
          "Flywheel/testLaunchSpeedRotPerSec",
          ShooterConstants.VELOCITIES.MINIMUM_LAUNCH_VELOCITY,
          Pair({ it.inRotationsPerSecond }, { it.rotations.perSecond }))

  private val m_sysIdRoutine =
      SysIdRoutine(
          SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4.0), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              Consumer { state: SysIdRoutineLog.State? ->
                run {
                  SignalLogger.writeString("state", state.toString())
                  CustomLogger.recordOutput("Flywheel/sysIdState", state.toString())
                }
              }),
          Mechanism(
              { volts: WPILibVoltage ->
                currentRequest = Request.FlywheelRequest.OpenLoop(volts.`in`(Volts).volts)
              },
              null,
              object : SubsystemBase("Flywheel") {}))

  init {
    if (RobotBase.isReal()) {
      io.configurePIDCurrent(
          ShooterConstants.PID.REAL_KP0,
          ShooterConstants.PID.REAL_KI0,
          ShooterConstants.PID.REAL_KD0,
          ShooterConstants.PID.REAL_KP1,
          ShooterConstants.PID.REAL_KI1,
          ShooterConstants.PID.REAL_KD1)
      io.configureFFCurrent(
          ShooterConstants.PID.REAL_KS0,
          ShooterConstants.PID.REAL_KV0,
          ShooterConstants.PID.REAL_KA0,
          ShooterConstants.PID.REAL_KS1,
          ShooterConstants.PID.REAL_KV1,
          ShooterConstants.PID.REAL_KA1)
    } else {
      io.configurePIDVoltage(
          ShooterConstants.PID.SIM_KP, ShooterConstants.PID.SIM_KI, ShooterConstants.PID.SIM_KD)
      io.configureFFVoltage(
          ShooterConstants.PID.SIM_KS, ShooterConstants.PID.SIM_KV, ShooterConstants.PID.SIM_KA)
    }
  }

  override fun onLoop() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Flywheel", inputs)
    CustomLogger.recordOutput(
        "Flywheel/targetAngularVelocityRPM", flywheelVelocityTarget.inRotationsPerMinute)
    CustomLogger.recordOutput("Flywheel/targetVoltage", flywheelVoltageTarget.inVolts)
    CustomLogger.recordOutput("Flywheel/currentState", currentState)
    CustomLogger.recordOutput("Flywheel/currentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Flywheel/isAtTargetedVelocity", isAtTargetedVelocity)

    var nextState = currentState

    when (currentState) {
      FlywheelState.UNINITIALIZED -> {
        nextState = fromFlywheelRequestToState(currentRequest)
      }
      FlywheelState.OPEN_LOOP -> {
        io.setVoltage(flywheelVoltageTarget)
        nextState = fromFlywheelRequestToState(currentRequest)
      }
      FlywheelState.TARGET_VELOCITY -> {
        io.setVelocity(flywheelVelocityTarget)
        nextState = fromFlywheelRequestToState(currentRequest)
      }
      FlywheelState.IDLE -> {
        flywheelVelocityTarget = ShooterConstants.VELOCITIES.IDLE_VELOCITY
        io.setVelocity(ShooterConstants.VELOCITIES.IDLE_VELOCITY)
        nextState = fromFlywheelRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }

  fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
    return m_sysIdRoutine.quasistatic(direction)
  }

  fun sysIdDynamic(direction: SysIdRoutine.Direction): Command {
    return m_sysIdRoutine.dynamic(direction)
  }

  companion object {
    enum class FlywheelState {
      UNINITIALIZED,
      IDLE,
      OPEN_LOOP,
      TARGET_VELOCITY
    }

    inline fun fromFlywheelRequestToState(request: Request.FlywheelRequest): FlywheelState {
      return when (request) {
        is Request.FlywheelRequest.TargetVelocity -> FlywheelState.TARGET_VELOCITY
        is Request.FlywheelRequest.OpenLoop -> FlywheelState.OPEN_LOOP
        is Request.FlywheelRequest.Idle -> FlywheelState.IDLE
      }
    }

    data class CalculatedLaunchData(
        val distanceToTarget: Length,
        val launchVelocity: LinearVelocity,
        val timeOfFlight: Time,
        val wantedRotation: Angle
    )

    fun calculateLaunchData(
        drivetrainPose: Pose2d,
        chassisSpeeds: ChassisSpeeds
    ): CalculatedLaunchData {
      return calculateLaunchData(
          drivetrainPose,
          chassisSpeeds,
          if (FieldConstants.inTrenchAllianceZone(drivetrainPose)) {
            FieldConstants.HUB_POSE
          } else {
            if (FieldConstants.inLeft(drivetrainPose)) {
              FieldConstants.PASSING_LEFT_TARGET
            } else {
              FieldConstants.PASSING_RIGHT_TARGET
            }
          })
    }

    fun calculateLaunchData(
        drivetrainPose: Pose2d,
        chassisSpeeds: ChassisSpeeds,
        targetTranslation: Translation3d
    ): CalculatedLaunchData {
      val rotatedFlywheel =
          ShooterConstants.SHOOTER_OFFSET.translation.rotateBy(drivetrainPose.rotation)
      val flywheelPosition = drivetrainPose.translation + rotatedFlywheel

      val targetHeight = targetTranslation.z

      // Calculate the flywheel's distance to the HUB
      val flywheelTTargetX = targetTranslation.x - flywheelPosition.x
      val flywheelTTargetY = targetTranslation.y - flywheelPosition.y
      val flywheelTTargetMag =
          sqrt(flywheelTTargetX.inMeters.pow(2) + flywheelTTargetY.inMeters.pow(2)).meters

      // Get field-relative drivetrain velocity, and convert it into a vector.
      val fieldSpeeds =
          ChassisSpeeds(
              edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
                  chassisSpeeds.chassisSpeedsWPILIB, drivetrainPose.rotation.inRotation2ds))

      // Flywheel tangential velocity
      val flywheelCurrentTransform =
          ShooterConstants.SHOOTER_OFFSET.translation.rotateBy(drivetrainPose.rotation)
      val flywheelSpeeds =
          Velocity2d(
                  (flywheelCurrentTransform.x * fieldSpeeds.omega.inRadiansPerSecond +
                          Constants.Universal.EPSILON.meters)
                      .perSecond,
                  (flywheelCurrentTransform.y * fieldSpeeds.omega.inRadiansPerSecond +
                          Constants.Universal.EPSILON.meters)
                      .perSecond)
              .rotateBy(90.degrees * fieldSpeeds.omega.sign)

      val driveVector =
          Vector(
              Matrix(
                  N2(),
                  N1(),
                  doubleArrayOf(
                      (fieldSpeeds.vx + flywheelSpeeds.x).inMetersPerSecond,
                      (fieldSpeeds.vy + flywheelSpeeds.y).inMetersPerSecond)))

      val robotTHubVector =
          Vector(
              Matrix(
                  N2(), N1(), doubleArrayOf(flywheelTTargetX.inMeters, flywheelTTargetY.inMeters)))

      // Get the distance (signed) between the robot and the HUB
      val hubUnitVector = robotTHubVector.times(1.0 / flywheelTTargetMag.inMeters)
      val parallelScalar = driveVector.dot(hubUnitVector).meters

      val a =
          (targetHeight.inMeters - ShooterConstants.SHOOTER_HEIGHT.inMeters) *
              ShooterConstants.SHOOTER_ANGLE.cos.pow(2) -
              ShooterConstants.SHOOTER_ANGLE.sin *
                  ShooterConstants.SHOOTER_ANGLE.cos *
                  flywheelTTargetMag.inMeters
      val b =
          2 *
              (targetHeight.inMeters - ShooterConstants.SHOOTER_HEIGHT.inMeters) *
              ShooterConstants.SHOOTER_ANGLE.cos *
              parallelScalar.inMeters -
              ShooterConstants.SHOOTER_ANGLE.sin *
                  flywheelTTargetMag.inMeters *
                  parallelScalar.inMeters
      val c =
          (targetHeight.inMeters - ShooterConstants.SHOOTER_HEIGHT.inMeters) *
              parallelScalar.inMeters.pow(2) +
              Constants.Universal.gravity.inMetersPerSecondPerSecond *
                  flywheelTTargetMag.inMeters.pow(2) / 2.0

      val launchSpeedFF = (flywheelTTargetMag.inMeters * 0.1).meters.perSecond
      val launchSpeed =
          max(
                  (-b + sqrt(b.pow(2) - 4.0 * a * c)) / (2 * a),
                  (-b - sqrt(b.pow(2) - 4.0 * a * c)) / (2 * a))
              .meters
              .perSecond + launchSpeedFF
      val launchSpeedField = launchSpeed * ShooterConstants.SHOOTER_ANGLE.cos
      val launchSpeedZ = launchSpeed * ShooterConstants.SHOOTER_ANGLE.sin

      val timeOfFlight =
          (flywheelTTargetMag.inMeters /
                  (launchSpeed.inMetersPerSecond * ShooterConstants.SHOOTER_ANGLE.cos +
                      parallelScalar.inMeters))
              .seconds

      val ballDistanceOffset = driveVector.times(timeOfFlight.inSeconds)

      val targetVirt =
          targetTranslation.toTranslation2d() -
              Translation2d(ballDistanceOffset.get(0).meters, ballDistanceOffset.get(1).meters)

      CustomLogger.recordOutput("Flywheel/targetVirt", targetVirt.translation2d)

      var theta = drivetrainPose.rotation
      for (i in 1..10) {
        val iterativeFlywheelPosition =
            drivetrainPose.translation + ShooterConstants.SHOOTER_OFFSET.translation.rotateBy(theta)
        var thetaNew =
            atan2(
                    (targetVirt.y - iterativeFlywheelPosition.y).inMeters,
                    (targetVirt.x - iterativeFlywheelPosition.x).inMeters)
                .radians

        if ((thetaNew - theta).absoluteValue < 1E-3.degrees) {
          theta = thetaNew
          break
        } else {
          theta = thetaNew
        }
      }

      CustomLogger.recordOutput("Flywheel/wantedRotDegs", theta.inDegrees)

      return CalculatedLaunchData(
          targetVirt.minus(drivetrainPose.translation).magnitude.meters,
          sqrt(launchSpeedField.inMetersPerSecond.pow(2) + launchSpeedZ.inMetersPerSecond.pow(2))
              .meters
              .perSecond,
          timeOfFlight,
          theta)
    }

    private val distanceToFlywheelMap: InterpolatingTreeMap<Length, AngularVelocity> =
        InterpolatingTreeMap(
            { startValue, endValue, q ->
              MathUtil.inverseInterpolate(startValue.value, endValue.value, q.value)
            },
            { startValue, endValue, t ->
              AngularVelocity(MathUtil.interpolate(startValue.value, endValue.value, t))
            })

    private val passingFlywheelMap: InterpolatingTreeMap<Length, AngularVelocity> =
        InterpolatingTreeMap(
            { startValue, endValue, q ->
              MathUtil.inverseInterpolate(startValue.value, endValue.value, q.value)
            },
            { startValue, endValue, t ->
              AngularVelocity(MathUtil.interpolate(startValue.value, endValue.value, t))
            })

    init {
      distanceToFlywheelMap.put(2.02.meters, 33.rotations.perSecond)
      distanceToFlywheelMap.put(2.26.meters, 35.5.rotations.perSecond)
      distanceToFlywheelMap.put(2.52.meters, 37.5.rotations.perSecond)
      distanceToFlywheelMap.put(2.76.meters, 40.rotations.perSecond)
      distanceToFlywheelMap.put(3.01.meters, 42.rotations.perSecond)
      distanceToFlywheelMap.put(3.27.meters, 44.rotations.perSecond)
      distanceToFlywheelMap.put(3.48.meters, 46.rotations.perSecond)
      distanceToFlywheelMap.put(3.71.meters, 48.rotations.perSecond)
      distanceToFlywheelMap.put(4.01.meters, 50.rotations.perSecond)
      distanceToFlywheelMap.put(4.35.meters, 52.rotations.perSecond)
      distanceToFlywheelMap.put(4.47.meters, 53.rotations.perSecond)
      distanceToFlywheelMap.put(4.80.meters, 55.rotations.perSecond)
      distanceToFlywheelMap.put(5.03.meters, 57.rotations.perSecond)

      passingFlywheelMap.put(2.meters, 30.rotations.perSecond)
      passingFlywheelMap.put(2.5.meters, 35.rotations.perSecond)
      passingFlywheelMap.put(3.meters, 40.rotations.perSecond)
      passingFlywheelMap.put(3.5.meters, 45.rotations.perSecond)
      passingFlywheelMap.put(4.meters, 50.rotations.perSecond)
    }

    fun distanceToFlywheelRPM(distanceToTarget: Length): AngularVelocity {
      if (2.02.meters <= distanceToTarget && distanceToTarget <= 5.03.meters) {
        return distanceToFlywheelMap.get(distanceToTarget)
      }
      return max(
          ShooterConstants.VELOCITIES.MINIMUM_LAUNCH_VELOCITY,
          min(
              (7.844 * distanceToTarget.inMeters + 18.04674).rotations.perSecond,
              ShooterConstants.VELOCITIES.MAXIMUM_LAUNCH_VELOCITY))
    }

    fun passingDistanceToFlywheelRPM(distanceToTarget: Length): AngularVelocity {
      if (2.meters <= distanceToTarget && distanceToTarget <= 4.meters) {
        return passingFlywheelMap.get(distanceToTarget)
      }
      return max(
          ShooterConstants.VELOCITIES.MINIMUM_LAUNCH_VELOCITY,
          min(
              (9.25752 * distanceToTarget.inMeters + 9.25).rotations.perSecond,
              ShooterConstants.VELOCITIES.MAXIMUM_LAUNCH_VELOCITY))
    }
  }
}
