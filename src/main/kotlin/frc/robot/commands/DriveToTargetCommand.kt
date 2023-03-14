package frc.robot.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.CommandBase

import frc.robot.Constants
import frc.robot.lib.*
import frc.robot.subsystems.*

class DriveToTargetCommand(
    drive: DriveSubsystem,
    goalDist: Double,
    targetHeight: Double,
    offset: Double
): CommandBase() {
    /** Creates a new DriveToTargetCommand. */
    val drive: DriveSubsystem = drive
    val camera: Camera = drive.camera
    val goalDist: Double = goalDist
    val targetHeight: Double = targetHeight
    val offset: Double = offset

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        // get the translation to the target from the camera
        val translationToTargetDist: Translation2d? =
            camera.getTranslationToTarget(camera.getTarget(true, 0), targetHeight)
        // nullsafe
        if (translationToTargetDist != null) {
            //turn the distances to motor speeds
            val translationToTargetSpeed =
                Translation2d(
                    MathUtil.applyDeadband((drive.driveController.calculate(translationToTargetDist.getX(), goalDist)), Constants.Swerve.stickDeadband),
                    MathUtil.applyDeadband(drive.driveController.calculate(translationToTargetDist.getY(), goalDist), Constants.Swerve.stickDeadband)
                )
            //drive in open-loop mode using the speed
            drive.drive(translationToTargetSpeed, 0.0, true, false)
        }
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        val translationToTarget: Translation2d? =
            camera.getTranslationToTarget(camera.getTarget(true, 0), targetHeight)
        if (translationToTarget != null) {
            // if we have a target, check distance to it.
            return translationToTarget.getDistance(Translation2d(0.0, 0.0)) < goalDist + offset
        }
        // if there's no target, exit the command
        return true
    }
}
