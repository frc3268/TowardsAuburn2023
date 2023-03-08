package frc.robot.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DriveSubsystem

class AutoBalanceCommand(drive: DriveSubsystem) : CommandBase() {
    /** Creates a new AutoBalanceCommand. */
    val drive: DriveSubsystem = DriveSubsystem()

    var gyroAngle: Double = 0.0
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
        gyroAngle = drive.getPitch().getDegrees()
    }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        gyroAngle = drive.getPitch().getDegrees()
        drive.drive(Translation2d(Math.sin(gyroAngle) * 1.5, 0.0), 0.0, true, false)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return gyroAngle > 5.0
    }
}
