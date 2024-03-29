package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import java.util.function.DoubleSupplier
import java.util.function.BooleanSupplier

import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem

class JoystickDriveCommand(
    val drive: DriveSubsystem,
    val y: DoubleSupplier,
    val x: DoubleSupplier,
    val tankMode: BooleanSupplier
) : CommandBase() {

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() { 
        /* Drive */
        drive.drive(
            y.getAsDouble()*0.7,
            x.getAsDouble(),
            if (tankMode.getAsBoolean()) Constants.DriveMode.TANK else Constants.DriveMode.ARCADE
        )
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) { }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
