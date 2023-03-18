package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase

import frc.robot.Constants
import frc.robot.lib.units.*
import frc.robot.subsystems.DriveSubsystem

class AutoBalanceCommand(drive: DriveSubsystem) : CommandBase() {
    val drive: DriveSubsystem = DriveSubsystem()
    var gyroAngle: Double = 0.rad

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
        gyroAngle = drive.getPitch().deg
    }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        gyroAngle = drive.getPitch().deg
        drive.drive(
            Math.sin(gyroAngle.rad) * 1.5,
            0.0,
            Constants.DriveMode.ARCADE
        )
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean =
        gyroAngle > 5.deg
}
