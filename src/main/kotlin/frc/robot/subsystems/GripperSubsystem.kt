package frc.robot.subsystems

import edu.wpi.first.wpilibj.motorcontrol.Talon
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

class GripperSubsystem : SubsystemBase() {
    val motor: Talon = Talon(Constants.limbs.gripperPort)
    val maxOutput: Double = 0.5
    var direction = -1

    init {
        motor.set(0.0)
    }

    fun setIn(): Command{
        return run{motor.set(maxOutput * direction)}.andThen(stopMotor())
    }

    fun setOut(): Command {
        return run{motor.set(1.0)}.andThen(stopMotor())
    }

    fun stopMotor(): Command{
        return run{motor.set(0.0)}
    }

    override fun periodic() {
    }

    override fun simulationPeriodic() {
    }

}