package com.team4099.robot2026

import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2026.auto.AutonomousSelector
import com.team4099.robot2026.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2026.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2026.config.ControlBoard
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.config.constants.FieldConstants
import com.team4099.robot2026.config.constants.VisionConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.drivetrain.GyroIOPigeon2
import com.team4099.robot2026.subsystems.drivetrain.GyroIOSim
import com.team4099.robot2026.subsystems.drivetrain.ModuleIOTalonFXReal
import com.team4099.robot2026.subsystems.drivetrain.ModuleIOTalonFXSim
import com.team4099.robot2026.subsystems.vision.Vision
import com.team4099.robot2026.subsystems.vision.camera.CameraIOPVSim
import com.team4099.robot2026.subsystems.vision.camera.CameraIOPhotonvision
import com.team4099.robot2026.util.driver.Jessika
import edu.wpi.first.wpilibj.RobotBase
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.smoothDeadband

object RobotContainer {
  private val drivetrain: Drive
  private val vision: Vision

  var driveSimulation: SwerveDriveSimulation? = null
  var isAligning = false

  init {
    SimulatedArena.overrideInstance(Arena2026Rebuilt(false))

    if (Constants.Universal.DISABLE_COLLISIONS)
        SimulatedArena.overrideInstance(FieldConstants.EMPTY_MAPLESIM_FIELD)

    if (RobotBase.isReal()) {
      drivetrain =
          Drive(
              GyroIOPigeon2,
              ModuleIOTalonFXReal.generateModules(),
              { edu.wpi.first.math.geometry.Pose2d.kZero },
              { pose -> {} })
      vision =
          Vision(
              *VisionConstants.CAMERAS.map {
                    CameraIOPhotonvision(
                        it.value.first,
                        it.key,
                        it.value.second,
                        drivetrain::addVisionMeasurement,
                        { drivetrain.rotation },
                        { drivetrain.chassisSpeeds })
                  }
                  .toTypedArray(),
              poseSupplier = { drivetrain.pose })

      when (Constants.Universal.whoami) {
        Constants.WHOAMI.CLONEBOT -> {
          // TODO - instantiate real subsystems here!
        }
      }
    } else {
      driveSimulation =
          SwerveDriveSimulation(Drive.mapleSimConfig, DrivetrainConstants.INITIAL_SIM_POSE)
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation)

      drivetrain =
          Drive(
              GyroIOSim(driveSimulation!!.gyroSimulation),
              ModuleIOTalonFXSim.generateModules(driveSimulation!!),
              driveSimulation!!::getSimulatedDriveTrainPose,
              driveSimulation!!::setSimulationWorldPose)

      vision =
          if (Constants.Universal.SIMULATE_VISION)
              Vision(
                  *VisionConstants.CAMERAS.map {
                        CameraIOPVSim(
                            it.value.first,
                            it.key,
                            it.value.second,
                            drivetrain::addVisionMeasurement,
                            { drivetrain.rotation },
                            { drivetrain.chassisSpeeds })
                      }
                      .toTypedArray(),
                  poseSupplier = { drivetrain.pose })
          else Vision(poseSupplier = { Pose2d() })

      // TODO - instantiate sim subsystems here!
    }
  }

  fun mapDefaultCommands() {
    drivetrain.defaultCommand =
        TeleopDriveCommand(
            driver = Jessika(),
            { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
            { ControlBoard.slowMode },
            drivetrain)
  }

  fun setDriveBrakeMode(neutralModeValue: NeutralModeValue = NeutralModeValue.Brake) {
    drivetrain.moduleIOs.forEach { it.toggleBrakeMode(neutralModeValue) }
  }

  fun mapTeleopControls() {
    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain))
  }

  fun mapTestControls() {}

  fun mapTunableCommands() {}

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain, vision)

  fun resetSimulationField() {
    if (!RobotBase.isSimulation()) return

    driveSimulation!!.setSimulationWorldPose(DrivetrainConstants.INITIAL_SIM_POSE)
    SimulatedArena.getInstance().resetFieldForAuto()
  }

  fun updateSimulation() {
    if (!RobotBase.isSimulation()) return

    SimulatedArena.getInstance().simulationPeriodic()
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation!!.simulatedDriveTrainPose)
    Logger.recordOutput(
        "FieldSimulation/Fuel", *SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"))
  }
}
