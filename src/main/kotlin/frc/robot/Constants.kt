package frc.robot

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import com.revrobotics.CANSparkMax.IdleMode;
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

    object Swerve {
        public val stickDeadband = 0.1;

        public val gyroID: Int = 6;
        public val invertGyro: Boolean = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public val trackWidth = Units.inchesToMeters(21.73);
        public val wheelBase = Units.inchesToMeters(21.73);
        public val wheelDiameter = Units.inchesToMeters(4.0);
        public val wheelCircumference = wheelDiameter * Math.PI;

        public val openLoopRamp: Double = 0.25;
        public val closedLoopRamp: Double = 0.0;

        public val driveGearRatio: Double = (8.14 / 1.0); // 6.75:1
        public val angleGearRatio: Double = (12.8 / 1.0); // 12.8:1

        public val swerveKinematics: SwerveDriveKinematics =
            SwerveDriveKinematics(
                Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Voltage Compensation */
        public val voltageComp: Double = 12.0;

        /* Swerve Current Limiting */
        public val angleContinuousCurrentLimit: Int = 20;
        public val driveContinuousCurrentLimit: Int = 80;

        /* Angle Motor PID Values */
        public val angleKP: Double = 0.01;
        public val angleKI: Double = 0.0;
        public val angleKD: Double = 0.0;
        public val angleKFF: Double = 0.0;

        /* Drive Motor PID Values */
        public val driveKP: Double = 0.1
        public val driveKI: Double  = 0.0
        public val driveKD: Double  = 0.0
        public val driveKFF: Double  = 0.0

        /* Drive Motor Characterization Values */
        public val driveKS: Double  = 0.667
        public val driveKV: Double = 2.44
        public val driveKA: Double  = 0.27

        /* Drive Motor Conversion Factors */
        public val driveConversionPositionFactor: Double  =
            (wheelDiameter * Math.PI) / driveGearRatio;
        public val driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public val angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        public val maxSpeed = 4.5; // meters per second
        public val maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public val angleNeutralMode: IdleMode = IdleMode.kBrake;
        public val driveNeutralMode: IdleMode = IdleMode.kBrake;

        /* Motor Inverts */
        public val driveInvert: Boolean = false;
        public val angleInvert: Boolean = false;

        /* Angle Encoder Invert */
        public val canCoderInvert: Boolean = false;

        /*Swerve Module Constants */
        //!FIX: ADD REAL CONSTANTS
        public val mod0: SwerveDriveModuleConstants = SwerveDriveModuleConstants(0, 0, 0, Rotation2d.fromDegrees(0.0))
        public val mod1: SwerveDriveModuleConstants = SwerveDriveModuleConstants(0, 0, 0, Rotation2d.fromDegrees(0.0))
        public val mod2: SwerveDriveModuleConstants = SwerveDriveModuleConstants(0, 0, 0, Rotation2d.fromDegrees(0.0))
        public val mod3: SwerveDriveModuleConstants = SwerveDriveModuleConstants(0, 0, 0, Rotation2d.fromDegrees(0.0))
    }
    data class SwerveDriveModuleConstants(
        val driveMotorID: Int,
        val angleMotorID: Int,
        val canCoderID: Int,
        val angleOffset: Rotation2d
    )
}
