package frc.robot.commands

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand
import frc.robot.subsystems.DriveSubsystem
import java.util.function.BiConsumer
import java.util.function.DoubleSupplier
import java.util.function.Supplier

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
class TurnAmountCommand(target: Double, drive: DriveSubsystem) :
        ProfiledPIDCommand(
                // The ProfiledPIDController used by the command
                drive.turnController,
                // This should return the measurement
                DoubleSupplier { drive.getYaw() },
                // This should return the goal (can also be a constant)
                Supplier { TrapezoidProfile.State(target, 0.0) },
                // This uses the output
                // does this shit even work?? idk
                BiConsumer { output: Double?, setpoint: TrapezoidProfile.State? ->
                    run { drive.driveArcadeConsumer({ 0.0 }, { output!! }) }
                }
        ) {
    init {
        getController().setTolerance(2.0)
        getController().enableContinuousInput(-180.0, 180.0)
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return getController().atGoal()
    }
}
