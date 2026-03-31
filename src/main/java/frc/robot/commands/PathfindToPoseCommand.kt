package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathfindingCommand
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.pathfinding.LocalADStar
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.DriveFeedforwards
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.Constants
import frc.robot.Constants.PathPlannerConstants
import frc.robot.subsystems.DriveSubsystem


class PathfindToPoseCommand(pose: Pose2d) : Command() {
    init {
        addRequirements(DriveSubsystem)
//        val command: PathfindingCommand = PathfindingCommand(
//        Pose2d(Translation2d(0.0,0.0), Rotation2d(0.0)),
//            Constants.DriveConstants.PATH_CONSTRAINTS,
//            DriveSubsystem::getPose,
//            DriveSubsystem::getCurrentSpeeds,
//            {
//                speeds: ChassisSpeeds, _: DriveFeedforwards ->
//                DriveSubsystem.drive(speeds, fieldRelative = true)
//            },
//            PPHolonomicDriveController(
//                // Translation PID
//                PIDConstants(
//                    PathPlannerConstants.TRANSLATION_P,
//                    PathPlannerConstants.TRANSLATION_I,
//                    PathPlannerConstants.TRANSLATION_D,
//                ),
//                // Rotation PID
//                PIDConstants(
//                    PathPlannerConstants.ROTATION_P,
//                    PathPlannerConstants.ROTATION_I,
//                    PathPlannerConstants.ROTATION_D,
//                ),
//                1.0,
//            ),
//            RobotConfig.fromGUISettings(),
//            DriveSubsystem::shouldFlipPath,
//            DriveSubsystem.subsystem,
//        )
    }

    val pose = pose

    override fun execute() {
        //Pathfinding.ensureInitialized()
        Pathfinding.setPathfinder(LocalADStar())
        CommandScheduler.getInstance().schedule(
            AutoBuilder.pathfindToPose(
                Pose2d(Translation2d(0.0,0.0), Rotation2d(0.0)),
                Constants.DriveConstants.PATH_CONSTRAINTS,
                0.0,
            ),
        )
    }

    override fun isFinished(): Boolean {
        return DriveSubsystem.getPose() == pose
    }

}