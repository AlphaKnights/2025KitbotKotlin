package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ClimbSubsystem

class ClimbCommand: Command() {
    init {
        addRequirements(ClimbSubsystem)
    }

    override fun execute() {
        super.execute()
        ClimbSubsystem.setPosition(1.0)
    }

    override fun isFinished(): Boolean {
        return super.isFinished()
        if ClimbSubsystem.
    }
}