package frc.robot.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem
import edu.wpi.first.math.geometry.Pose2d


class DriveSetPointCommand(
    private val X: () ->  Double,
    private val Y: () -> Double,
    private val Angle: () -> Double
) : Command() {

    init {
        addRequirements(DriveSubsystem)
    }
    val setX = X()
    val setY = Y()
    val setAngle = Angle()

    // Do angle optimization (south) and scalable tuning based on max speed
    override fun execute() {
        super.execute()
        // take current rotation in radians and make a new PID Controller
        val curpose = DriveSubsystem.getPose()
        val rotateController = PIDController(0.005, 0.0, 0.01)
        val driveController = PIDController(0.5, 0.01, 0.01)
//        val dir = when {
//            (curpose > Math.PI/2)  -> Math.PI
//            (curpose < -Math.PI/2) -> -Math.PI
//            else -> 0.0
//        }

        // set PID deadzones and angle wrapping
        rotateController.setTolerance(Rotation2d.fromDegrees(10.0).radians)
        rotateController.enableContinuousInput(-Math.PI, Math.PI)

        driveController.setTolerance(0.01) // meters


        // calculate rotational speed using PID controller, making sure max speed is respected
        val rotSpeed =
            clamp(
                rotateController.calculate(curpose.rotation.radians, setAngle),
                -1.0,
                1.0,
            ) * Constants.DriveConstants.MAX_ANGULAR_SPEED

        val driveSpeedX =
            clamp(
                driveController.calculate(curpose.translation.x, setX),
                -1.0,
                1.0,
            ) * Constants.DriveConstants.MAX_METERS_PER_SECOND

        val driveSpeedY =
            clamp(
                driveController.calculate(curpose.translation.y, setY),
                -1.0,
                1.0,
            ) * Constants.DriveConstants.MAX_METERS_PER_SECOND

        DriveSubsystem.drive(
            ChassisSpeeds(
                driveSpeedX,
                driveSpeedY,
                rotSpeed,
            ),
            fieldRelative = false,
        )
    }

    override fun isFinished(): Boolean {
        val curpose = DriveSubsystem.getPose()
        return if (
            curpose.x > setX - 0.05 &&
            curpose.x < setX + 0.05 &&
            curpose.y > setY - 0.05 &&
            curpose.y < setY + 0.05
//            curpose.rotation.radians > setAngle + 0.05 &&
//            curpose.rotation.radians < setAngle - 0.05
        ) {
            true
        } else false
    }

}