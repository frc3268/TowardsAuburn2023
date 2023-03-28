package frc.robot.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand
import frc.robot.subsystems.DriveSubsystem
import java.util.function.BiConsumer
import java.util.function.DoubleSupplier
import java.util.function.Supplier

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
class DriveAmountCommand(targetMeters: Double, drive: DriveSubsystem) : ProfiledPIDCommand(
    // The ProfiledPIDController used by the command
    drive.forwardController,
    // This should return the measurement
    DoubleSupplier { drive.getAverageEncoderDistance() },
    // This should return the goal (can also be a constant)
    Supplier { TrapezoidProfile.State(targetMeters, 0.0) },
    // This uses the output
    BiConsumer { output: Double?, setpoint: TrapezoidProfile.State? ->
        drive.driveArcadeConsumer({ 0.0 }, { output!! })
    },
    drive
) {
    init {
        getController().setTolerance(0.1)
        getController().enableContinuousInput(0.0, 54.0)
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return getController().atGoal()
    }
}
