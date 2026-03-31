
/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import frc.robot.Constants.AimingConstants
import kotlin.math.sqrt
import kotlin.math.pow

object DriveToArcPoseGenerator {
    fun generatePath(): Pose2d {
        val curpose = DriveSubsystem.getPose()
//
        //val hubPos = Translation2d(AimingConstants.BLUE_HUB_X, AimingConstants.BLUE_HUB_Y)
        val distanceHubX = curpose.translation.x - AimingConstants.BLUE_HUB_X // distance between robot and hub
        val distanceHubY = curpose.translation.y - AimingConstants.BLUE_HUB_Y
        val scalar = AimingConstants.DISTANCE / sqrt(distanceHubX.pow(2.0)+distanceHubY.pow(2.0))// creates a scalar to find a position at the right distance and direction from the hub (hub relative)
        val targetX = AimingConstants.BLUE_HUB_X + (distanceHubX * scalar) // finds the field relative position of the scaled vector
        val targetY = AimingConstants.BLUE_HUB_Y + (distanceHubY * scalar)

        return Pose2d(Translation2d(targetX,targetY), curpose.rotation)

    }
}
