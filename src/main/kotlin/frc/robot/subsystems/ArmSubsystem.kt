package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase

import frc.robot.Constants
import frc.robot.lib.units.*

class ArmSubsystem() : SubsystemBase() {
    val motor: CANSparkMax = CANSparkMax(ArmConsts.motorPort, MotorType.kBrushless)
    val encoder: RelativeEncoder = motor.getEncoder()

    init {
        encoder.setPositionConversionFactor(360 / (147 / 1.0))
    }

    override fun periodic() {
    }

    override fun simulationPeriodic() {
    }
}