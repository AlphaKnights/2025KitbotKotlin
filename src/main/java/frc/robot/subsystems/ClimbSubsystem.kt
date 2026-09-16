package frc.robot.subsystems

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ClimbConstants

object ClimbSubsystem: SubsystemBase() {

    private val motor = TalonFX(67, ClimbConstants.CANBUS)

    private var targetPosition = 0.0

    init {
        val config = TalonFXConfiguration().apply {
            Slot0.apply {
                kP = 0.01
                kI = 0.0
                kD = 0.001
                kG = 0.01
                kV = 0.0
            }

            CurrentLimits.apply {
                SupplyCurrentLimitEnable = true
                SupplyCurrentLimit = ClimbConstants.SUPPLY_CURRENT_LIMIT
            }

            SoftwareLimitSwitch.apply {
                ForwardSoftLimitEnable = true
                ForwardSoftLimitThreshold = ClimbConstants.UPPER_LIMIT
                ReverseSoftLimitEnable = true
                ReverseSoftLimitThreshold = ClimbConstants.LOWER_LIMIT
            }

            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        }
        motor.configurator.apply(config)
    }

    fun setPosition(position: Double) {
        // motor.setPosition(position)
        targetPosition = position
        motor.setControl(PositionVoltage(position).withSlot(0))
    }

    fun setSpeed(up: Boolean) {
        if (up) motor.set(ClimbConstants.SPEED)
        else motor.set(-ClimbConstants.SPEED)
    }

    fun stop() {
        motor.stopMotor()
    }

    fun isInPosition(): Boolean =
        (targetPosition - ClimbConstants.DEADZONE) < motor.closedLoopError.valueAsDouble < (targetPosition + ClimbConstants.DEADZONE)

}
