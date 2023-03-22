package frc.robot

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units

import com.revrobotics.CANSparkMax.IdleMode

import frc.robot.lib.units.*

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. inside the companion object). Do not put anything functional in this class.
 *
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
class Constants {
    object OperatorConstants {
        const val kDriverControllerPort = 0
    }

    object Field {
        public val chargeStationPoint = Pose2d(
            Translation2d(Units.inchesToMeters(224.0), Units.inchesToMeters(84.25)), Rotation2d.fromDegrees(0.deg)
        )
        public val startingPose = Pose2d(
            Translation2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)), Rotation2d.fromDegrees(180.deg))
    }

    object Camera {
        //!FixME
        public val camHeight = 10.0.inches
        public val cameraAngle = 45.0.deg
    }

    enum class DriveMode{
        ARCADE, TANK
    }

    object Drive{
        val leftFrontID = 1
        val leftBackID = 2
        val rightFrontID = 3
        val rightBackID = 4
        val startYaw: Double = 0.0.deg

        //ramsete params
        val ksVolts:Double = 0.14087
        val kvVoltSecondsPerMeter:Double = 1.3489
        val kaVoltSecondsSquaredPerMeter:Double = 0.096935

        val kPDriveVel:Double = 1.4502
        val kTrackWidthMeters:Double = Units.inchesToMeters(18.0)
        val kDriveKinematics:DifferentialDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)

        val kMaxSpeedMetersPerSeconds:Double = 3.0
        val kMaxAccelerationMetersPerSecondSquared = 1.0

        val kRamseteB:Double = 2.0
        val kRamseteZeta:Double = 0.7

        
    }

    object LimelightPipelineIndexes {
        val reflectiveTape = 0
        val aprilTag = 1
    }

    data class TankDirection(
        val forward:Double,
        val rot: Double
    )
}
