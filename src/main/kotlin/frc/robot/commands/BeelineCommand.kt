package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand

import frc.robot.Constants
import frc.robot.subsystems.*

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
class BeelineCommand(drive: DriveSubsystem, target:Pose2d) : InstantCommand() {
    val drive: DriveSubsystem = drive
    val target:Pose2d = target
    init {
        addRequirements(drive)
    }
    // Called when the command is initially scheduled.
    override fun initialize() {
        val config: TrajectoryConfig =
            TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
            )
            .setKinematics(Constants.Swerve.swerveKinematics)

        val distanceToGo: Translation2d =
            Translation2d(
                target.translation.getX() - drive.swervePoseEstimator.estimatedPosition.translation.getX(),
                target.translation.getY() - drive.swervePoseEstimator.estimatedPosition.translation.getY()
            )
        val rotationToGo: Rotation2d =
            drive.swervePoseEstimator.estimatedPosition.rotation.minus(
                target.rotation
            )

        // tajectory towards the set translation
        val trajectory: Trajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing any direction
                Pose2d(0.0, 0.0, drive.getYaw()),
                // No int. waypoints
                listOf(),
                // end at our change station point
                Pose2d(distanceToGo, rotationToGo),
                config
            )

        var thetaController: ProfiledPIDController =
            ProfiledPIDController(
                Constants.AutoConstants.kPThetaController,
                0.0,
                0.0,
                Constants.AutoConstants.kThetaControllerConstraints
            )
        thetaController.enableContinuousInput(-Math.PI, Math.PI)

        val swerveControllerCommand: SwerveControllerCommand =
            SwerveControllerCommand(
                trajectory,
                drive::getPose,
                Constants.Swerve.swerveKinematics,
                PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0),
                PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0),
                thetaController,
                { drive::setModuleStates },
                drive
            )
        run {
            drive.resetOdometry(trajectory.initialPose)
            swerveControllerCommand
            // might have to reset odometry again
        }
    }
}
