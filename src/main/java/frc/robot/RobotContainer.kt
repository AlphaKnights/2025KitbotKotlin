/*
 * (C) 2025 Galvaknights
 */
package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import frc.robot.commands.*
import frc.robot.commands.autoalign.AutoAlignAutoCommand
import frc.robot.commands.autoalign.AutoAlignManualCommand
import frc.robot.commands.coralmanipulator.IntakeCommand
import frc.robot.commands.coralmanipulator.LaunchCommand
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.DriveToArcPoseGenerator
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.subsystems.ArcSlidingCalc

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer {
    private val joystickController = JoystickController()
    private val buttonBoard =
        CommandJoystick(
            Constants.OperatorConstants.BUTTON_BOARD_PORT,
        )

    init {
        /*LimelightSubsystem.startPolling()

        NamedCommands.registerCommands(
            mapOf(
                "Left" to
                    AutoAlignAutoCommand(
                        Constants.AlignDirection.LEFT,
                    ),
                "Right" to
                    AutoAlignAutoCommand(
                        Constants.AlignDirection.RIGHT,
                    ),
                "Lvl 1" to
                    ElevatorPosAutoCommand(
                        Constants.ElevatorConstants.LVL_1_HEIGHT,
                    ),
                "Lvl 2" to
                    ElevatorPosAutoCommand(
                        Constants.ElevatorConstants.LVL_2_HEIGHT,
                    ),
                "Lvl 3" to
                    ElevatorPosAutoCommand(
                        Constants.ElevatorConstants.LVL_3_HEIGHT,
                    ),
                "Lvl 4" to
                    ElevatorPosAutoCommand(
                        Constants.ElevatorConstants.LVL_4_HEIGHT,
                    ),
                "Intake" to IntakeCommand(),
                "Delivery" to LaunchCommand(),
            ),
        )
        */
        configureBindings()
    }

    private fun configureBindings() {
        // Drive control
        DriveSubsystem.defaultCommand =
            DriveCommand(
                x = { joystickController.x() },
                y = { joystickController.y() },
                rot = { joystickController.rot() },
                autoAngle = { joystickController.getRawButton(Constants.OperatorConstants.AIMING_BUTTON) }
            )

        // Reset heading
        joystickController
            .heading()
            .whileTrue(
                ResetHeadingCommand(),
            )

        // Auto Align
        joystickController
            .alignL().whileTrue(
                AutoAlignManualCommand(
                    Constants.AlignDirection.LEFT,
                ),
            )

        /*joystickController
            .alignR().whileTrue(
                AutoAlignManualCommand(
                    Constants.AlignDirection.RIGHT,
                ),
            )*/

        /*joystickController
            .north().whileTrue(
                NorthCommand(
                    x = { joystickController.x() },
                    y = { joystickController.y() },
                )
            )*/

        joystickController.resetOdometry()
            .whileTrue(
                ResetOdometry()
            )

        joystickController
            .driveToArc().onTrue(
                DriveSetPointCommand(
                    { DriveToArcPoseGenerator.generatePath().x } ,
                    { DriveToArcPoseGenerator.generatePath().y },
                    { -DriveToArcPoseGenerator.generatePath().rotation.radians }
                )
            )

        joystickController
            .slideLeft().whileTrue(
                DriveCommand(
                    {ArcSlidingCalc.getXChange(Constants.DriveConstants.MAX_ANGULAR_SPEED*Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE)},
                    {ArcSlidingCalc.getYChange(Constants.DriveConstants.MAX_ANGULAR_SPEED*Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE)},
                    {Constants.DriveConstants.MAX_ANGULAR_SPEED*Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE},
                    {false}
                )
            )

        joystickController
            .slideRight().whileTrue(
                DriveCommand(
                    {ArcSlidingCalc.getXChange(-Constants.DriveConstants.MAX_ANGULAR_SPEED*Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE)},
                    {ArcSlidingCalc.getYChange(-Constants.DriveConstants.MAX_ANGULAR_SPEED*Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE)},
                    {-Constants.DriveConstants.MAX_ANGULAR_SPEED*Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE},
                    {false}
                )
            )

//        configureElevatorBindings()
//        configureCoralManipulatorBindings()
    }

    /**private fun configureElevatorBindings() {
    // Manual elevator control
    buttonBoard
    .button(
    Constants.OperatorConstants.ELEVATOR_UP_BUTTON,
    ).whileTrue(
    ElevatorManualCommand(
    Constants.ElevatorDirection.UP,
    ),
    )

    buttonBoard
    .button(
    Constants.OperatorConstants.ELEVATOR_DOWN_BUTTON,
    ).whileTrue(
    ElevatorManualCommand(
    Constants.ElevatorDirection.DOWN,
    ),
    )

    // Elevator positioning
    buttonBoard
    .button(
    Constants.OperatorConstants.ELEVATOR_LVL_1_BUTTON,
    ).onTrue(
    ElevatorPosCommand(
    Constants.ElevatorConstants.LVL_1_HEIGHT,
    ),
    )

    buttonBoard
    .button(
    Constants.OperatorConstants.ELEVATOR_LVL_2_BUTTON,
    ).onTrue(
    ElevatorPosCommand(
    Constants.ElevatorConstants.LVL_2_HEIGHT,
    ),
    )

    buttonBoard
    .button(
    Constants.OperatorConstants.ELEVATOR_LVL_3_BUTTON,
    ).onTrue(
    ElevatorPosCommand(
    Constants.ElevatorConstants.LVL_3_HEIGHT,
    ),
    )

    buttonBoard
    .button(
    Constants.OperatorConstants.ELEVATOR_LVL_4_BUTTON,
    ).onTrue(
    ElevatorPosCommand(
    Constants.ElevatorConstants.LVL_4_HEIGHT,
    ),
    )
    }

    private fun configureCoralManipulatorBindings() {
    buttonBoard
    .button(
    Constants.OperatorConstants.DELIVERY_BUTTON,
    ).onTrue(
    LaunchCommand(),
    )
    buttonBoard
    .button(
    Constants.OperatorConstants.INTAKE_BUTTON,
    ).onTrue(
    IntakeCommand(),
    )
    }
     */
    fun getAutonomousCommand(): Command {

        return PathPlannerAuto("test")
    }


}