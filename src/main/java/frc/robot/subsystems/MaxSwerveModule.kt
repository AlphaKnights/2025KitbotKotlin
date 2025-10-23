// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkBase.ControlType
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkClosedLoopController
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.Constants.ModuleConstants
import kotlin.math.PI


class MAXSwerveModule(drivingCANId: Int, turningCANId: Int, chassisAngularOffset: Rotation2d) {
    private val m_drivingSpark = SparkMax(drivingCANId, SparkLowLevel.MotorType.kBrushless)
    private val m_turningSpark = SparkMax(turningCANId, SparkLowLevel.MotorType.kBrushless)

    private val m_drivingEncoder: RelativeEncoder = m_drivingSpark.encoder
    private val m_turningEncoder: AbsoluteEncoder = m_turningSpark.absoluteEncoder

    private val m_drivingClosedLoopController: SparkClosedLoopController = m_drivingSpark.closedLoopController
    private val m_turningClosedLoopController: SparkClosedLoopController = m_turningSpark.closedLoopController

    private var m_chassisAngularOffset = 0.0
    private var m_desiredState = SwerveModuleState(0.0, Rotation2d())


    private val drivingConfig: SparkMaxConfig = SparkMaxConfig()

    private val turningConfig: SparkMaxConfig = SparkMaxConfig()

    // Use module constants to calculate conversion factors and feed forward gain.
    private val drivingFactor = ModuleConstants.kWheelDiameterMeters * PI / ModuleConstants.kDrivingMotorReduction;
    private val turningFactor = 2 * PI;
    private val drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps
    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    init {
        // Apply the respective configurations to the SPARKS. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle.
        drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        drivingConfig.encoder
            .positionConversionFactor(drivingFactor) // meters
            .velocityConversionFactor(drivingFactor / 60.0); // meters per second
        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(0.04, 0.0, 0.0)
            .velocityFF(drivingVelocityFeedForward)
            .outputRange(-1.0, 1.0);

        turningConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);
        turningConfig.absoluteEncoder
            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .inverted(true)
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0); // radians per second
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(1.0, 0.0, 0.0)
            .outputRange(-1.0, 1.0)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0.0, turningFactor);


        m_drivingSpark.configure(
            drivingConfig, SparkBase.ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )
        m_turningSpark.configure(
            turningConfig, SparkBase.ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        m_chassisAngularOffset = chassisAngularOffset.getDegrees()
        m_desiredState.angle = Rotation2d(m_turningEncoder.position)
        m_drivingEncoder.setPosition(0.0)
    }

    val state: SwerveModuleState
        /**
         * Returns the current state of the module.
         *
         * @return The current state of the module.
         */
        get() =// Apply chassis angular offset to the encoder position to get the position
            // relative to the chassis.
            SwerveModuleState(
                m_drivingEncoder.velocity,
                Rotation2d(m_turningEncoder.position - m_chassisAngularOffset)
            )

    val position: SwerveModulePosition
        /**
         * Returns the current position of the module.
         *
         * @return The current position of the module.
         */
        get() =// Apply chassis angular offset to the encoder position to get the position
            // relative to the chassis.
            SwerveModulePosition(
                m_drivingEncoder.position,
                Rotation2d(m_turningEncoder.position - m_chassisAngularOffset)
            )

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    fun setDesiredState(desiredState: SwerveModuleState) {
        // Apply chassis angular offset to the desired state.
        val correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset))

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(Rotation2d(m_turningEncoder.position))

        // Command driving and turning SPARKS towards their respective setpoints.
        m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity)
        m_turningClosedLoopController.setReference(correctedDesiredState.angle.radians, ControlType.kPosition)

        m_desiredState = desiredState
    }

    /** Zeroes all the SwerveModule encoders.  */
    fun resetEncoders() {
        m_drivingEncoder.setPosition(0.0)
    }
}