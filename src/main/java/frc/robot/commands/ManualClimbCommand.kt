package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ClimbSubsystem

class ManualClimbCommand(private val direction: Boolean): Command() {

    init {
        addRequirements(ClimbSubsystem)
    }

    override fun execute() {
        ClimbSubsystem.setSpeed(direction)
    }
}