/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.AimingCalc
import frc.robot.subsystems.DriveSubsystem

class DriveCommand(
    private val left: () -> Double,
    private val right: () -> Double,
) : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        super.execute()
        println("DRIVING AT left = $left, right = $right")

        DriveSubsystem.differentialDrivetrain.tankDrive(left(), right())

    }

    override fun end(interrupted: Boolean) {
        DriveSubsystem.differentialDrivetrain.tankDrive(0.0, 0.0)
    }
}