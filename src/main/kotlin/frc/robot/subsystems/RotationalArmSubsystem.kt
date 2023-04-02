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
import edu.wpi.first.wpilibj2.command.Subsystem

import kotlin.math.cos

import frc.robot.Constants
import java.util.function.DoubleSupplier

class RotationalArmSubsystem : Subsystem {
    val motor: CANSparkMax = CANSparkMax(Constants.limbs.RotationalArm.motorPort, MotorType.kBrushless)
    val encoder: RelativeEncoder = motor.encoder

    init {
        //measure encoder rortations to some easy to convert portion of 360 deg, then set conv. factor to 360 / computed encoder value per 1 rotation
        encoder.setPositionConversionFactor(360 / (147 / 1.0))
        //or an offset possibly?
        encoder.position = 0.0
    }

    override fun periodic() {
        SmartDashboard.putNumber("Extension Motor Rotations", getMeasurement())  
    }

    override fun simulationPeriodic() {
    }

    fun getMeasurement(): Double {
        return encoder.position
    }

    fun unz():Command{
        return run{motor.set(-0.3)} 
    }

    fun dnz():Command{
        return run{motor.set(0.3)}
    }

    fun up():Command {
        return run{motor.set(-0.3)}.until({getMeasurement() < 5 }).andThen(stop())
    }

    fun down():Command{
        return run{motor.set(0.3)}.until({getMeasurement() > 150 }).andThen(stop())
    }

    fun amtn(target:Double):Command{
        val x = getMeasurement()
        return run{motor.set(0.3)}.until({Math.abs(x+target - getMeasurement()) < 5}).andThen(stop())
    }

    fun stop():Command{
        return run{motor.set(0.0)}
    }

} 