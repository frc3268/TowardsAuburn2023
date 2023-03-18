package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.MathUtil

import java.util.function.BooleanSupplier

import frc.robot.Constants
import frc.robot.lib.units.*
import frc.robot.subsystems.DriveSubsystem

class DriveUntilConditionCommand(
    drive: DriveSubsystem,
    condition: BooleanSupplier,
    fieldOriented: Boolean,
    forward: Boolean
) : CommandBase() {
    val drive: DriveSubsystem = drive
    val condition: BooleanSupplier = condition
    val fieldOriented: Boolean = fieldOriented
    val forward: Int = if (forward) 1 else -1

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        drive.drive(
            if (fieldOriented) forward.toDouble() else forward * Math.sin(drive.getYaw().deg.rad),
            0.0,
            Constants.DriveMode.ARCADE
        )
     }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) { }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return condition.getAsBoolean()
    }
}
