package frc.robot.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand
import java.util.function.BiConsumer
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import frc.robot.Constants
import frc.robot.subsystems.RotationalArmSubsystem

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
class SetRotAngleCommand(arm: RotationalArmSubsystem, target: Double) : ProfiledPIDCommand(
    // The ProfiledPIDController used by the command
    ProfiledPIDController(
        Constants.limbs.RotationalArm.kp,
        Constants.limbs.RotationalArm.ki,
        Constants.limbs.RotationalArm.kd,
        TrapezoidProfile.Constraints(Constants.limbs.RotationalArm.kmaxspeed, Constants.limbs.RotationalArm.kmaxaccel)
    ),
        // This should return the measurement
        DoubleSupplier { arm.getMeasurement() },
        // This should return the goal (can also be a constant)
        Supplier { TrapezoidProfile.State(target, 0.0) },
        // This uses the output
        BiConsumer { output: Double?, setpoint: TrapezoidProfile.State? -> 
            
        },
        arm
    ) {

    init{
        getController().setTolerance(5.0)
        getController().enableContinuousInput(0.0, 360.0)
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return controller.atGoal()
    }
}
