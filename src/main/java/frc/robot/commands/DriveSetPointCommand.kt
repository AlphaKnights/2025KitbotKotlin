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
    private val setX: Double,
    private val setY: Double,
    private val setAngle: Double
) : Command() {

    init {
        addRequirements(DriveSubsystem)
    }
    // Do angle optimization (south) and scalable tuning based on max speed
    override fun execute() {
        super.execute()
        // take current rotation in radians and make a new PID Controller
        val curpose = DriveSubsystem.getPose()
        val rotateController = PIDController(0.005, 0.0, 0.01)
        val driveController = PIDController(0.1, 0.0, 0.005)
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
            fieldRelative = true,
        )
    }

    override fun isFinished(): Boolean {
        return DriveSubsystem.getPose().equals(Pose2d(setX,setY, Rotation2d.fromRadians(setAngle)))
    }

}