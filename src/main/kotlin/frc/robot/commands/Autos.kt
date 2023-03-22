package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
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

        public fun beelineAuto(target: Pose2d, drive: DriveSubsystem): Command {
            val volateConstraint: DifferentialDriveVoltageConstraint =
                    DifferentialDriveVoltageConstraint(
                            SimpleMotorFeedforward(
                                    Constants.Drive.ksVolts,
                                    Constants.Drive.kvVoltSecondsPerMeter,
                                    Constants.Drive.kaVoltSecondsSquaredPerMeter
                            ),
                            Constants.Drive.kDriveKinematics,
                            10.0
                    )
            val trajectoryConfig: TrajectoryConfig =
                    TrajectoryConfig(
                                    Constants.Drive.kMaxSpeedMetersPerSeconds,
                                    Constants.Drive.kaVoltSecondsSquaredPerMeter
                            )
                            .setKinematics(Constants.Drive.kDriveKinematics)
                            .addConstraint(volateConstraint)

            val trajectory: Trajectory =
                    TrajectoryGenerator.generateTrajectory(
                            listOf(drive.getPose(), target),
                            trajectoryConfig
                    )

            val ramseteCommand: RamseteCommand =
                    RamseteCommand(
                            trajectory,
                            drive::getPose,
                            RamseteController(
                                    Constants.Drive.kRamseteB,
                                    Constants.Drive.kRamseteZeta
                            ),
                            SimpleMotorFeedforward(
                                    Constants.Drive.ksVolts,
                                    Constants.Drive.kvVoltSecondsPerMeter,
                                    Constants.Drive.kaVoltSecondsSquaredPerMeter
                            ),
                            Constants.Drive.kDriveKinematics,
                            drive::getWheelSpeeds,
                            PIDController(Constants.Drive.kPDriveVel, 0.0, 0.0),
                            PIDController(Constants.Drive.kPDriveVel, 0.0, 0.0),
                            drive::tankDriveVolts,
                            drive
                    )
            return ramseteCommand.andThen({drive.tankDriveVolts(0.0, 0.0)}, drive)
        }
    }
}
