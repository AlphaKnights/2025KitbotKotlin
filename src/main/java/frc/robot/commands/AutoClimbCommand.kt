package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ClimbSubsystem
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.awaitAll

/**
* Direction = true for up, false for down
 */
class AutoClimbCommand(private val direction: Boolean): Command() {

    init {
        addRequirements(ClimbSubsystem)
        withTimeout(5.0)
    }

    override fun execute() {
        super.execute()
        if (direction) {
            ClimbSubsystem.setPosition(1.0)
        }
        else ClimbSubsystem.setPosition(0.0)
    }

    override fun isFinished(): Boolean = ClimbSubsystem.isInPosition()

}