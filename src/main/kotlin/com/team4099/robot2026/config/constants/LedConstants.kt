package com.team4099.robot2026.config.constants

import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.signals.RGBWColor
import edu.wpi.first.wpilibj.util.Color

/** CODE CLONE NOTE(nathan): Don't change this file. */
object LedConstants {
  const val START_INDEX = 8
  const val END_INDEX = 399

  enum class CandleState(val request: ControlRequest) {
    NOTHING(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kGhostWhite))),
    BLUE_DISABLED(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kNavy))),
    RED_DISABLED(SolidColor(START_INDEX, END_INDEX).withColor(RGBWColor(Color.kDarkRed))),
  }
}
