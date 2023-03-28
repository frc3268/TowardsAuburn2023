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

// TODO Update code to be relevant to extension arm
class ExtensionArmSubsystem : ProfiledPIDSubsystem(
    ProfiledPIDController(
        /* TODO Replace constants */
        Constants.limbs.ExtensionArm.kp,
        Constants.limbs.ExtensionArm.ki,
        Constants.limbs.ExtensionArm.kd,
        TrapezoidProfile.Constraints(Constants.limbs.ExtensionArm.kmaxspeed, Constants.limbs.ExtensionArm.kmaxaccel)
    )
) {
    val motor: CANSparkMax = CANSparkMax(Constants.limbs.ExtensionArm.motorPort, MotorType.kBrushless)
    val encoder: RelativeEncoder = motor.getEncoder()

    init {
        //Measure rotations for 1 full extension. conversion factor should be 100 / that number
        encoder.setPositionConversionFactor(1.0/1.0)
        //encoder starts at 0
        encoder.position = 0.0
    }

    override fun periodic() {
        SmartDashboard.putNumber("Extension Motor Rotations", measurement)
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

    fun setExtensionPercent(percent: Double): Command {
        return runOnce {
            setGoal(percent)
            enable()
        }
    }
}

