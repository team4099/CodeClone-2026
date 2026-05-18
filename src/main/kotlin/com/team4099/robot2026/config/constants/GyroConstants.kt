package com.team4099.robot2026.config.constants

import org.team4099.lib.units.derived.degrees

/** CODE CLONE NOTE(nathan): Don't change this file. */
object GyroConstants {
  val mountPitch
    get() =
        when (Constants.Universal.whoami) {
          Constants.WHOAMI.CODE_CLONE -> 0.degrees
        }

  val mountRoll
    get() =
        when (Constants.Universal.whoami) {
          Constants.WHOAMI.CODE_CLONE -> 0.degrees
        }

  val mountYaw
    get() =
        when (Constants.Universal.whoami) {
          Constants.WHOAMI.CODE_CLONE -> 0.degrees
        }
}
