package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.MathUtil
import frc.robot.subsystems.DriveSubsystem
import frc.robot.Constants
import java.util.function.BooleanSupplier
import frc.robot.lib.units.*

class DriveUntilConditionCommand (drive:DriveSubsystem, condition:BooleanSupplier, fieldOriented:Boolean, forward:Boolean): CommandBase() {
    /**
     * Creates a new DriveUntilConditionCommand.
     */
    val drive:DriveSubsystem = drive
    val condition:BooleanSupplier = condition
    val fieldOriented:Boolean = fieldOriented
    val forward:Int = if (forward) { 1 } else{ -1 }
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        drive.drive(
            Translation2d(
                MathUtil.applyDeadband(0.5 * forward, 0.0),
                Constants.Swerve.stickDeadband
            ),
            0.deg,
            true,
            fieldOriented
        )
     }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) { }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return condition.getAsBoolean()
    }
}
