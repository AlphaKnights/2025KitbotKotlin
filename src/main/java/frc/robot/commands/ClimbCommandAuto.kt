package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.ClimbSubsystem

/**
* Direction = true for up, false for down
 */
class ClimbCommandAuto(private val direction: Boolean): Command() {

    init {
        addRequirements(ClimbSubsystem)
        //withTimeout(10.0)
    }

    override fun execute() {
        super.execute()
        if (direction) {
            ClimbSubsystem.setPosition(Constants.ClimbConstants.LEVEL_ONE_POSITION)
        }
        else ClimbSubsystem.setPosition(0.0)
    }

    override fun isFinished(): Boolean = ClimbSubsystem.isInPosition()

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
        ClimbSubsystem.stop()
    }

}