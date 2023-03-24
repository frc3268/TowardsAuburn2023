package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.RelativeEncoder
import edu.wpi.first.wpilibj2.command.SubsystemBase

import frc.robot.Constants
import frc.robot.lib.units.*

class ArmSubsystem() : SubsystemBase() {
    val motor: CANSparkMax = CANSparkMax(0, MotorType.kBrushless)
    val encoder: RelativeEncoder = motor.getEncoder()

    init {
        //MUST MUST MUST BE CHANGED TO CORRECT THING
        encoder.setPositionConversionFactor(360 / (147 / 1.0))
    }

    override fun periodic() {
    }

    override fun simulationPeriodic() {
    }
}