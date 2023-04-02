package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

import frc.robot.Constants
import frc.robot.lib.units.*
import frc.robot.subsystems.DriveSubsystem

class AutoBalanceCommand(val drive: DriveSubsystem) : CommandBase() {
    var gyroAngle: Double = 0.0

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
        gyroAngle = drive.getPitch()
    }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        gyroAngle = drive.getPitch()
        drive.drive(
            0.0,
            Math.sin(gyroAngle) * 180/Math.PI,
            Constants.DriveMode.ARCADE
        )
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) { }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return (Math.abs(gyroAngle) < 5.0)
    }
}
