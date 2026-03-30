/*
 * (C) 2025 Galvaknights
 */
package frc.robot

import edu.wpi.first.math.MathUtil.applyDeadband
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger

@Suppress("MagicNumber")
class JoystickController :
    Joystick(
        Constants.OperatorConstants.DRIVER_CONTROLLER_PORT,
    ) {
    fun x(): Double =
        (
                -applyDeadband(
                    getRawAxis(1),
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * (-getRawAxis(3) + 1) / 2
                )

    fun y(): Double =
        (
                -applyDeadband(
                    getRawAxis(0),
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * (-getRawAxis(3) + 1) / 2
                )

    fun rot(): Double =
        (
                -applyDeadband(
                    getRawAxis(2),
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * (-getRawAxis(3) + 1) / 2
                )

    fun heading(): Trigger =
        JoystickButton(
            this,
            Constants.OperatorConstants.RESET_HEADING_BUTTON,
        )

    fun alignL(): Trigger =
        JoystickButton(
            this,
            Constants.OperatorConstants.ALIGN_LEFT_BUTTON,
        )

    fun alignR(): Trigger =
        JoystickButton(
            this,
            Constants.OperatorConstants.ALIGN_RIGHT_BUTTON,
        )

    fun north(): Trigger =
        JoystickButton(
            this,
            Constants.OperatorConstants.NORTH_BUTTON
        )

    fun driveToArc(): Trigger =
        JoystickButton(
            this,
            Constants.OperatorConstants.ARC_BUTTON
        )

}
