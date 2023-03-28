package frc.robot.subsystems

import edu.wpi.first.wpilibj.motorcontrol.Talon
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

class GripperSubsystem : SubsystemBase() {
    val motor: Talon = Talon(Constants.limbs.gripperPort)
    val maxOutput: Double = 0.5
    var direction = 1

    init {
        motor.set(0.0)
    }

    fun setIn() {
        motor.set(maxOutput * direction)
    }

    fun setOut() {
        motor.set(-1 * maxOutput * direction)
    }

    fun stopMotor() {
        motor.set(0.0)
    }

    override fun periodic() {
    }

    override fun simulationPeriodic() {
    }

}