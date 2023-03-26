package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command

import kotlin.math.cos

import frc.robot.Constants

// TODO Update code to be relevant to extension arm
class ExtensionArmSubsystem : ProfiledPIDSubsystem(
    ProfiledPIDController(
        /* TODO Replace constants */
        Constants.limbs.RotationalArm.kp,
        Constants.limbs.RotationalArm.ki,
        Constants.limbs.RotationalArm.kd,
        TrapezoidProfile.Constraints(Constants.limbs.RotationalArm.kmaxspeed, Constants.limbs.RotationalArm.kmaxaccel)
    )
) {
    val motor: CANSparkMax = CANSparkMax(Constants.limbs.RotationalArm.motorPort, MotorType.kBrushless)
    val encoder: RelativeEncoder = motor.getEncoder()

    init {
        //MUST MUST MUST BE CHANGED TO CORRECT THING
        encoder.setPositionConversionFactor(360 / (147 / 1.0))
        //or an offset possibly?
        encoder.position = 0.0
    }

    override fun periodic() {
    }

    override fun simulationPeriodic() {
    }

    override fun getMeasurement(): Double {
        return encoder.position
    }

    override fun useOutput(output: Double, setpoint: TrapezoidProfile.State?) {
        //feedforward is irrelevant; gravicty does not need to be compensated for
        motor.set(output)
    }

    fun setToAngle(angle: Double): Command {
        return runOnce {
            setGoal(angle)
            enable()
        }
    }
}

