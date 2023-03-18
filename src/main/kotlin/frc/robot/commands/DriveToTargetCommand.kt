package frc.robot.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.CommandBase

import frc.robot.Constants
import frc.robot.lib.Camera
import frc.robot.lib.units.*
import frc.robot.subsystems.DriveSubsystem

class DriveToTargetCommand(
    drive: DriveSubsystem,
    goalDist: Double,
    targetHeight: Double,
    offset: Double
) : CommandBase() {
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
        val tankdirtotarget: Constants.TankDirection? =
            camera.getTankDirectionToTarget(camera.getTarget(true, 0), targetHeight)
        // nullsafe
        if (tankdirtotarget != null) {
            drive.drive(tankdirtotarget.forward, tankdirtotarget.rot, Constants.DriveMode.ARCADE)
        }
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        val translationToTarget: Constants.TankDirection? =
            camera.getTankDirectionToTarget(camera.getTarget(true, 0), targetHeight)
        if (translationToTarget != null) {
            // if we have a target, check distance to it.
            return translationToTarget.forward < goalDist + offset
        }
        // if there's no target, exit the command
        return true
    }
}
