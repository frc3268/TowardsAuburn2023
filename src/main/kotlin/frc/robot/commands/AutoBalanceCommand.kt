package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem
import frc.robot.units.*

class AutoBalanceCommand(drive: DriveSubsystem) : CommandBase() {
    val drive: DriveSubsystem = DriveSubsystem()
    var gyroAngle: Double = 0.rad

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
        gyroAngle = drive.getPitch().getRadians()
    }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        gyroAngle = drive.getPitch().getRadians()
        drive.drive(
                Translation2d(
                        MathUtil.applyDeadband(Math.sin(gyroAngle) * 1.5, 0.0),
                        Constants.Swerve.stickDeadband
                ),
                0.0,
                true,
                false
        )
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return gyroAngle > 5.deg
    }
}
