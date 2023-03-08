package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.math.geometry.Translation2d
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.CameraSubsystem

class DriveToTargetCommand (drive:DriveSubsystem, camera:CameraSubsystem goalDistI:Double, targetHeightI:Double, offset:Double): CommandBase() {
    /**
     * Creates a new DriveToTargetCommand.
     */
    val drive:DriveSubsystem = drive
    val camera:CameraSubsystem = camera
    var toTarget:Translation2d = Translation2d(0.0, 0.0)
    val goalDistI:Double = goalDistI
    val targetHeightI:Double = targetHeightI
    val offset:Double = offset
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive, camera)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
     }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() { 
        toTarget = camera.getTranslationToTarget(camera.?getTarget(true, 0.0), targetHeightI)
        val translation = Translation2d(drive.driveController.calculate(toTarget.getX(), goalDistI), drive.driveController.calculate(toTarget.getY(), goalDistI))
        drive.drive(translation, 0.0, true, false)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) { }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return toTarget.getDistance(Translation2d(0.0, 0.0)) < goalDistI+offset
    }
}
