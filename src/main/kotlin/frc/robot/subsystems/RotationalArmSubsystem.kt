package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command

import kotlin.math.cos

import frc.robot.Constants

class RotationalArmSubsystem : ProfiledPIDSubsystem(
    ProfiledPIDController(
        Constants.limbs.RotationalArm.kp,
        Constants.limbs.RotationalArm.ki,
        Constants.limbs.RotationalArm.kd,
        TrapezoidProfile.Constraints(Constants.limbs.RotationalArm.kmaxspeed, Constants.limbs.RotationalArm.kmaxaccel)
    )
) {
    val motor: CANSparkMax = CANSparkMax(Constants.limbs.RotationalArm.motorPort, MotorType.kBrushless)
    val encoder: RelativeEncoder = motor.encoder

    init {
        //measure encoder rortations to some easy to convert portion of 360 deg, then set conv. factor to 360 / computed encoder value per 1 rotation
        encoder.setPositionConversionFactor(360 / (147 / 1.0))
        //or an offset possibly?
        encoder.position = 0.0
    }

    override fun periodic() {
        SmartDashboard.putNumber("Extension Motor Rotations", measurement)
    }

    override fun simulationPeriodic() {
    }

    override fun getMeasurement(): Double {
        return (encoder.position )
    }

    override fun useOutput(output: Double, setpoint: TrapezoidProfile.State?) {
        if (setpoint != null) {
            //add feed forward
            val ffscalar = cos(Units.degreesToRadians(setpoint.position)) * Constants.limbs.RotationalArm.kff
            motor.set(output + ffscalar)
        }
    }

    fun setToAngle(angle: Double): Command {
        return runOnce {
            setGoal(angle)
            enable()
        }
    }
}