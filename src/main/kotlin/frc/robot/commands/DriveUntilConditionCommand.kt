package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.MathUtil

import java.util.function.BooleanSupplier

import frc.robot.Constants
import frc.robot.lib.units.*
import frc.robot.subsystems.DriveSubsystem

class DriveUntilConditionCommand(
    val drive: DriveSubsystem,
    val condition: BooleanSupplier,
    val fieldOriented: Boolean,
    forward: Boolean
) : CommandBase() {
    val forward: Double = if (forward) 0.7 else -0.7

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        drive.drive(
            0.0,
            0.7,
            Constants.DriveMode.ARCADE
        )
     }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) { }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
