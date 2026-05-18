package com.team4099.robot2026.subsystems.superstructure

import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.vision.Vision
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Superstructure(
    private val drivetrain: Drive,
    private val vision: Vision,
) : SubsystemBase() {
  companion object {
    enum class SuperstructureStates {}
  }
}
