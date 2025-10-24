/*
 * (C) 2025 Galvaknights
 */
package frc.robot

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

import com.revrobotics.spark.config.SparkBaseConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units

@Suppress("MagicNumber", "SpreadOperator")
object Constants {
    object OperatorConstants {
        const val DRIVER_CONTROLLER_PORT = 1
        const val DRIVE_DEADBAND = 0.1

        const val RESET_HEADING_BUTTON = 11

        const val ALIGN_LEFT_BUTTON = 5
        const val ALIGN_RIGHT_BUTTON = 7

        const val BUTTON_BOARD_PORT = 2
        const val ELEVATOR_UP_BUTTON = 8
        const val ELEVATOR_DOWN_BUTTON = 10

        const val ELEVATOR_LVL_1_BUTTON = 1
        const val ELEVATOR_LVL_2_BUTTON = 2
        const val ELEVATOR_LVL_3_BUTTON = 3
        const val ELEVATOR_LVL_4_BUTTON = 4

        const val INTAKE_BUTTON = 6
        const val DELIVERY_BUTTON = 9
    }

    object DriveConstants {
        const val MAX_METERS_PER_SECOND = 15
        const val MAX_ANGULAR_SPEED = 20

        private val TRACK_WIDTH = Units.inchesToMeters(26.5)
        private val WHEEL_BASE = Units.inchesToMeters(26.5)

        private val MODULE_POSITIONS =
            arrayOf(
                Translation2d(
                    WHEEL_BASE / 2.0,
                    TRACK_WIDTH / 2.0,
                ),
                Translation2d(
                    WHEEL_BASE / 2.0,
                    -TRACK_WIDTH / 2.0,
                ),
                Translation2d(
                    -WHEEL_BASE / 2.0,
                    TRACK_WIDTH / 2.0,
                ),
                Translation2d(
                    -WHEEL_BASE / 2.0,
                    -TRACK_WIDTH / 2.0,
                ),
            )

        /** The directions of the wheels when locking into an X formation */
        val MODULE_X_ROTATIONS =
            mapOf(
                "frontLeft" to
                    Rotation2d.fromDegrees(
                        45.0,
                    ),
                "frontRight" to
                    Rotation2d.fromDegrees(
                        -45.0,
                    ),
                "rearLeft" to
                    Rotation2d.fromDegrees(
                        -45.0,
                    ),
                "rearRight" to
                    Rotation2d.fromDegrees(
                        45.0,
                    ),
            )

        val DRIVE_KINEMATICS =
            SwerveDriveKinematics(*MODULE_POSITIONS)

        val FRONT_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d =
            Rotation2d
                .fromRotations(
                    -0.764892578125,
                )
        val FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d =
            Rotation2d
                .fromRotations(
                    0.75,
                )
        val BACK_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d =
            Rotation2d
                .fromRotations(
                    0.079833984375,
                )
        val BACK_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d =
            Rotation2d
                .fromRotations(
                    0.367919921875,
                )

        const val FRONT_LEFT_DRIVING_ID = 2
        const val REAR_LEFT_DRIVING_ID = 4
        const val FRONT_RIGHT_DRIVING_ID = 3
        const val REAR_RIGHT_DRIVING_ID = 5

        const val FRONT_LEFT_TURNING_ID = 14
        const val REAR_LEFT_TURNING_ID = 22
        const val FRONT_RIGHT_TURNING_ID = 1
        const val REAR_RIGHT_TURNING_ID = 12

       // const val FRONT_LEFT_CANCODER_ID = 3
        //const val REAR_LEFT_CANCODER_ID = 4
       // const val FRONT_RIGHT_CANCODER_ID = 2
        //const val REAR_RIGHT_CANCODER_ID = 1
    }

    object ModuleConstants {
        const val kDrivingMotorPinionTeeth: Int = 14

        // Calculations required for driving motor conversion factors and feed forward
        val kDrivingMotorFreeSpeedRps: Double = 5676.0 / 60
        const val kWheelDiameterMeters: Double = 0.0762
        const val kWheelCircumferenceMeters: Double = kWheelDiameterMeters * Math.PI
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        const val kDrivingMotorReduction: Double = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)

        val kDriveWheelFreeSpeedRps: Double = ((kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction)
    }

    object ElevatorConstants {
        const val LEFT_MOTOR_CAN_ID = 4
        const val RIGHT_MOTOR_CAN_ID = 32

        const val P = 0.08
        const val I = 0.0
        const val D = 0.0

        val IDLE_MODE = SparkBaseConfig.IdleMode.kCoast

        const val FORWARD_SOFT_LIMIT = 100.0
        const val REVERSE_SOFT_LIMIT = -100.0

        const val CURRENT_LIMIT = 40

        const val MAX_ELEVATOR_SPEED = 3.0
        const val MANUAL_SPEED_FACTOR = 0.1

        const val POS_DEADZONE = 5.0

        const val LVL_1_HEIGHT = 0.0
        const val LVL_2_HEIGHT = 47.0
        const val LVL_3_HEIGHT = 76.0
        const val LVL_4_HEIGHT = 50.0
    }

    enum class ElevatorDirection {
        UP,
        DOWN,
    }

    object LimelightConstants {
        /** How often the limelight should poll for data in seconds */
        const val POLLING_RATE = 20L

        /** How long the limelight should wait for a response in seconds */
        const val TIMEOUT = 500L // milliseconds

        /** The IP address of the limelight */
        const val IP_ADDR = "172.29.0.1"
    }

    object AlignConstants {
        const val ALIGN_DEADZONE = 0.03
        val ALIGN_ROT_DEADZONE = Units.degreesToRadians(5.0)

        const val FINE_ALIGN_DEADZONE = 1.0
        val FINE_ALIGN_ROT_DEADZONE =
            Units
                .degreesToRadians(
                    5.0,
                )

        const val MAX_SPEED = 1.0
        const val MAX_ANGULAR_SPEED = 1.0
        const val MIN_ANGULAR_SPEED = 0.2

        const val LEFT_X_OFFSET = -0.1625
        const val LEFT_Z_OFFSET = 0.457

        const val RIGHT_X_OFFSET = 0.1625
        const val RIGHT_Z_OFFSET = 0.457

        const val ALIGN_TIMEOUT = 5 // seconds
        const val ALIGN_SEEK_TIMEOUT = 1 // seconds

        const val ALIGN_TAG_SEEK_SPEED = 5.0
    }

    enum class AlignDirection {
        LEFT,
        RIGHT,
    }

    object UltrasonicConstants {
        /** The gap between the ultrasonic sensor and the coral inside the manipulator */
        const val CORAL_DISTANCE = 3

        /** The trig pin of the ultrasonic sensor corresponding to the SIG DIO pin */
        const val PING_CHANNEL = 1

        /** The echo pin of the ultrasonic sensor corresponding to the SIG DIO pin */
        const val ECHO_CHANNEL = 0
    }

    object LaunchConstants {
        const val MOTOR_ID = 30

        /** The power factor of the manipulator motor when in taking a coral */
        const val INTAKE_SPEED = 0.3

        /** The power factor of the manipulator motor when launching a coral */
        const val LAUNCH_SPEED = 1.0

        /** The amount of time the manipulator motor should wait until it stops running.
         *
         * This will stop the motor running if too much time has passed with the ultrasonic sensor detecting a
         * coral inside.
         *
         * This is to ensure that the motor doesn't draw too much power.
         * */
        const val LAUNCH_TIMEOUT = 3.0 // seconds

        /** The amount of time the manipulator motor should wait until it stops running.
         *
         * This will stop the motor running if too much time has passed with the ultrasonic sensor not detecting a
         * coral inside.
         *
         * This is to ensure that the motor doesn't draw too much power.
         * */
        const val INTAKE_TIMEOUT = 10.0 // seconds
    }

    object PathPlannerConstants {
        const val TRANSLATION_P = 5.0
        const val TRANSLATION_I = 0.0
        const val TRANSLATION_D = 0.0

        const val ROTATION_P = 17.0
        const val ROTATION_I = 0.0
        const val ROTATION_D = 0.0
    }
}
