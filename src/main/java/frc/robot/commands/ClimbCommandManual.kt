package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ClimbSubsystem

class ClimbCommandManual(private val direction: Boolean): Command() {

    init {
        addRequirements(ClimbSubsystem)
    }

    override fun execute() {
        ClimbSubsystem.setSpeed(direction)
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
        ClimbSubsystem.stop()
    }

}