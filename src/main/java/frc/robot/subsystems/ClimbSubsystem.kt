package frc.robot.subsystems



import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

object ClimbSubsystem: SubsystemBase() {

    private val motor = TalonFX(67, CANBus("didy"))

    var isInPosition: Boolean = motor.closedLoopError.valueAsDouble == 0.1

    init {
        val config = TalonFXConfiguration().apply {
            Slot0.apply {
                kP = 0.01
                kI = 0.0
                kD = 0.0001
                kG = 0.01
                kV = 0.0
            }

            CurrentLimits.apply {
                SupplyCurrentLimitEnable = true
                SupplyCurrentLimit = 120.0
            }

            SoftwareLimitSwitch.apply {
                ForwardSoftLimitEnable = true
                ForwardSoftLimitThreshold = 10.0
                ReverseSoftLimitEnable = true
                ReverseSoftLimitThreshold = 0.0
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
        motor.setControl(PositionVoltage(position).withSlot(0))
    }

    fun setSpeed(up: Boolean) {
        if (up) motor.set(2.0)
        else motor.set(-2.0)
    }

    fun stop() {
        motor.stopMotor()
    }

}
