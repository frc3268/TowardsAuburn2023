package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import java.util.function.DoubleSupplier
import java.util.function.BooleanSupplier

import frc.robot.Constants
import frc.robot.subsystems.*

class  JoystickDriveCommand(drive: DriveSubsystem, translationX: DoubleSupplier, translationY: DoubleSupplier, rotation:DoubleSupplier, fieldOriented: BooleanSupplier): CommandBase() {
    /**
     * Creates a new .
     */
    val drive: DriveSubsystem = drive
    val translationX: DoubleSupplier = translationX
    val translationY: DoubleSupplier = translationY
    val rotation: DoubleSupplier = rotation
    val fieldOriented: BooleanSupplier = fieldOriented

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() { 
        /* Get Values, Deadband*/
        val translationVal: Double = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.Swerve.stickDeadband)
        val strafeVal: Double = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.Swerve.stickDeadband)
        val rotationVal: Double = MathUtil.applyDeadband(rotation.getAsDouble(), Constants.Swerve.stickDeadband)
 
        /* Drive */
        drive.drive(
            Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            fieldOriented.getAsBoolean(), 
            true
        )
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) { }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
