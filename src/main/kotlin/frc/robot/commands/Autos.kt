package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.RamseteCommand
import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem

class Autos private constructor() {
    init {
        throw UnsupportedOperationException("This is a utility class!")
    }

    companion object {
        /** Example static factory for an autonomous command. */
        fun exampleAuto(): CommandBase {
            return Commands.sequence()
        }

        fun beelineAuto(target: Pose2d, drive: DriveSubsystem): Command {
            val forward = drive.getPose().translation.getDistance(target.translation)
            val rot = Math.atan((drive.getPose().translation.x - target.translation.x) / (drive.getPose().translation.y - target.translation.y))
            return TurnAmountCommand(rot, drive).andThen(DriveAmountCommand(drive.getAverageEncoderDistance() + forward, drive))
        }

        fun basicAuto(drive: DriveSubsystem) : Command {
            return DriveUntilConditionCommand(drive, {drive.getPitch() > 10.0}, false, false).withTimeout(2.0).andThen(AutoBalanceCommand(drive))
        }
    }
}
