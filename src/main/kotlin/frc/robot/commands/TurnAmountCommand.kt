package frc.robot.commands

import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.subsystems.DriveSubsystem
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
class TurnAmountCommand(targetDeg: Double, drive: DriveSubsystem) :
        PIDCommand(
                // The controller that the command will use
                drive.turnController,
                // This should return the measurement
                DoubleSupplier { drive.getYaw() },
                // This should return the setpoint (can also be a constant)
                DoubleSupplier { targetDeg },
                // This uses the output
                DoubleConsumer { output: Double -> { drive.driveArcadeConsumer({0.0}, {output}) } }
        ) {
    init{
        getController().setTolerance(2.0)
        getController().enableContinuousInput(-180.0, 180.0);
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return getController().atSetpoint()
    }
}
