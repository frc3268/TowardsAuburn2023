package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d

import frc.robot.subsystems.DriveSubsystem
import frc.robot.lib.units.*

class TurnAmountCommand(
    drive: DriveSubsystem,
    angle: Double,
    fieldOriented: Boolean
) : CommandBase() {
    val drive: DriveSubsystem = drive
    val angle: Double = angle
    val fieldOriented: Boolean = fieldOriented

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() { 
        drive.drive(Translation2d(0.0, 0.0), angle, false, fieldOriented)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) { }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return Math.abs(drive.getYaw().degrees - angle) > 5.deg
    }
}
