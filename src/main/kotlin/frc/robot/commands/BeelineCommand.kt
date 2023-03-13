package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.Trajectory
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.DriveSubsystem

class BeelineCommand (drive: DriveSubsystem): CommandBase() {
    /**
     * Creates a new BeelineCommand.
     */
    val drive:DriveSubsystem = drive
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {

     }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() { }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) { }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
