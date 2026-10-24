/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.ctre.phoenix6.hardware.Pigeon2
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.DriveFeedforwards
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkFlex
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkFlexConfig
import com.studica.frc.AHRS
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.DriveConstants
import frc.robot.Constants.PathPlannerConstants
import edu.wpi.first.networktables.DoublePublisher
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import frc.robot.Constants

object DriveSubsystem : SubsystemBase() {
    private var leftLeader: SparkFlex = SparkFlex(Constants.DriveConstants.FRONT_LEFT_DRIVING_ID, SparkLowLevel.MotorType.kBrushless)
    private var leftFollower: SparkFlex = SparkFlex(Constants.DriveConstants.REAR_LEFT_DRIVING_ID, SparkLowLevel.MotorType.kBrushless)
    private var rightLeader: SparkFlex = SparkFlex(Constants.DriveConstants.FRONT_RIGHT_DRIVING_ID, SparkLowLevel.MotorType.kBrushless)
    private var rightFollower: SparkFlex = SparkFlex(Constants.DriveConstants.REAR_RIGHT_DRIVING_ID, SparkLowLevel.MotorType.kBrushless)

    val differentialDrivetrain: DifferentialDrive = DifferentialDrive(leftLeader::set, rightLeader::set)
    //    private var gyro: Pigeon2 = Pigeon2(9)
    private var gyro: AHRS = AHRS(AHRS.NavXComType.kMXP_SPI)
//    private var odometry: SwerveDriveOdometry

    private val config: RobotConfig = RobotConfig.fromGUISettings()

    var counter = 0

    private val leftFollowerConfig: SparkFlexConfig = SparkFlexConfig().apply {
        follow(leftLeader)
    }
    private val rightFollowerConfig: SparkFlexConfig = SparkFlexConfig().apply {
        follow(rightLeader)
    }

    init {
//        gyro.enableBoardlevelYawReset(false)
        gyro.reset()

//        odometry =
//            DifferentialDriveOdometry(,  )

          //PathPlanner auto builder

        leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
      }


    override fun periodic() {
        // This method will be called once per scheduler run
//        odometry.update()

        resetOdometry(LimelightSubsystem.getPose()?.toPose2d() ?: getPose())
        println(getPose())
        //println(gyro.rotation2d)
        println("odometry = ${getPose()}")
        println("Arc Pose = ${DriveToArcPoseGenerator.generatePath()}")
    }


    fun resetPose(pose: Pose2d) {
        resetOdometry(pose)
    }

    fun resetOdometry(pose: Pose2d) {
//        odometry.resetPosition(
//            Rotation2d.fromDegrees(gyro.yaw.toDouble()), //gyro.getRotation2d(),
//            arrayOf(
//                frontLeft.getPosition(),
//                frontRight.getPosition(),
//                rearLeft.getPosition(),
//                rearRight.getPosition(),
//            ),
//            pose,
//        )
    }


    fun shouldFlipPath(): Boolean =
        (DriverStation.getAlliance() ?: DriverStation.Alliance.Red) == DriverStation.Alliance.Red

    @Suppress("MagicNumber")


    /**
     * Sets the swerve modules to a specific angle for X formation.
     *
     * This is used to prevent the robot from moving.
     */


    /**
     * Resets the gyro to 0 degrees.
     *
     * This is used to reset the heading of the robot.
     */
    fun zeroHeading() {
        gyro.reset()
    }

    fun getPose(): Pose2d {
//        return odometry.poseMeters
        return Pose2d.kZero
    }

}
