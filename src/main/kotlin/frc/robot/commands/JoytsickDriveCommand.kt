package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import java.util.function.DoubleSupplier
import java.util.function.BooleanSupplier

import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem

class JoystickDriveCommand(
    drive: DriveSubsystem,
    y: DoubleSupplier,
    x: DoubleSupplier,
    goblinMode: BooleanSupplier
) : CommandBase() {
    val drive: DriveSubsystem = drive
    val y: DoubleSupplier = y
    val x: DoubleSupplier = x
    val goblinMode: BooleanSupplier = goblinMode

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
            y.getAsDouble(),
            x.getAsDouble(),
            if(goblinMode.getAsBoolean()) Constants.DriveMode.GOBLIN else Constants.DriveMode.ARCADE
        )
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) { }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
