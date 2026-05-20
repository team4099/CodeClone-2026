package com.team4099.robot2026.config.constants

import com.team4099.robot2026.config.constants.Constants.WHOAMI
import com.team4099.robot2026.subsystems.vision.camera.CameraIO
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.centi
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.perSecond

object VisionConstants {
  val CONTROLLER_RUMBLE_DIST = 2.25.meters

  val BLUE_TARGET_TAGS = arrayOf<Int>()
  val RED_TARGET_TAGS = arrayOf<Int>()

  val AMBIGUITY_THESHOLD = .4
  val CONFIDENCE_THRESHOLD = 0.75
  val TAG_TRUST_THRESHOLD = 0.85

  // Pose acceptance thresholds
  val Z_MINIMUM = -10.centi.meters
  val Z_MAXIMUM = 8.inches + 10.centi.meters
  val POSE_ACCEPTANCE_MAX_LINEAR_SPEED = 3.meters.perSecond
  val POSE_ACCEPTANCE_MAX_ANGULAR_SPEED = 225.degrees.perSecond

  val CAMERAS: Map<String, Pair<CameraIO.DetectionPipeline, Transform3d>>
    get() =
        when (Constants.Universal.whoami) {
          WHOAMI.CLONEBOT -> mapOf()
        }

  // x, y, θ
  val singleTagStdDevs: Matrix<N3?, N1?> = VecBuilder.fill(0.05, 0.05, 9_999.0)
  val multiTagStdDevs: Matrix<N3?, N1?> = VecBuilder.fill(0.01, 0.01, 0.1)

  enum class OBJECT_CLASS(val id: Int, val mapleSimType: String?) {
    FUEL(0, "Fuel")
  }
}
