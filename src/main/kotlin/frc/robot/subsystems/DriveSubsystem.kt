package frc.robot.subsystems

import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.RelativeEncoder

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

import frc.robot.Constants
import frc.robot.lib.Camera
import frc.robot.lib.units.*

import java.util.function.DoubleSupplier
import org.photonvision.EstimatedRobotPose

class DriveSubsystem(private val startingPose: Pose2d) : SubsystemBase() {
    val gyro: AHRS = AHRS(SPI.Port.kMXP)

    // Controllers
    private val driveLeftFront: CANSparkMax =
        CANSparkMax(Constants.Drive.leftFrontID, MotorType.kBrushless)
    private val driveLeftBack: CANSparkMax =
        CANSparkMax(Constants.Drive.leftBackID, MotorType.kBrushless)
    private val driveRightFront: CANSparkMax =
        CANSparkMax(Constants.Drive.rightFrontID, MotorType.kBrushless)
    private val driveRightBack: CANSparkMax =
        CANSparkMax(Constants.Drive.rightBackID, MotorType.kBrushless)

    // Encoders
    val leftEncoder: RelativeEncoder = driveLeftFront.getEncoder()
    val rightEncoder: RelativeEncoder = driveRightFront.getEncoder()

    // Groups
    private val driveLeft: MotorControllerGroup =
        MotorControllerGroup(driveLeftFront, driveLeftBack)
    private val driveRight: MotorControllerGroup =
        MotorControllerGroup(driveRightFront, driveRightBack)

    // Drive
    private val drive: DifferentialDrive = DifferentialDrive(driveLeft, driveRight)

    // PID
    val linearP: Double = 0.05
    val linearD: Double = 0.03
    val forwardController: ProfiledPIDController = ProfiledPIDController(linearP, 0.0, linearD, TrapezoidProfile.Constraints(Constants.Drive.kMaxSpeedMetersPerSeconds, Constants.Drive.kMaxAccelerationMetersPerSecondSquared))

    val angularP: Double = 0.021
    val angularD: Double = 0.02
    val turnController = ProfiledPIDController(angularP, 0.0, angularD, TrapezoidProfile.Constraints(100.0, 300.0))

    val camera: Camera = Camera()

    // Odometry
    private val odometry: DifferentialDriveOdometry
    val poseEstimator: DifferentialDrivePoseEstimator

    init {
        //odometry
        zeroGyro()
        odometry =
            DifferentialDriveOdometry(
                gyro.getRotation2d(),
                leftEncoder.getPosition(),
                leftEncoder.getPosition(),
                startingPose
            )
        poseEstimator = DifferentialDrivePoseEstimator(Constants.Drive.kDriveKinematics, gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), startingPose)
        // config for motors
        driveLeftBack.restoreFactoryDefaults()
        driveLeftFront.restoreFactoryDefaults()
        driveRightFront.restoreFactoryDefaults()
        driveRightBack.restoreFactoryDefaults()
        driveLeftBack.setOpenLoopRampRate(0.9)
        driveLeftFront.setOpenLoopRampRate(0.9)
        driveRightBack.setOpenLoopRampRate(0.9)
        driveRightFront.setOpenLoopRampRate(0.9)

        // set encoder conversion factors-diameter of each wheen is 6 inches
        /* We need to verify that these conversion factors are correct. -- Weiju */
        leftEncoder.setPositionConversionFactor(12.0 / 1.0)
        rightEncoder.setPositionConversionFactor(12.0 / 1.0)
        leftEncoder.setVelocityConversionFactor((12.0 / 1.0) / (60 / 1))
        rightEncoder.setVelocityConversionFactor((12.0 / 1.0) / (60 / 1))
        resetEncoders()
    }

    override fun periodic() {
        camera.frame = camera.limelight.getLatestResult()
        SmartDashboard.putNumber("Encoder Dist", getAverageEncoderDistance())
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /*
       drive method:
           args taken:
           - arg1: y speed, left speed, y speed(respective to drive modes)
           - arg2: rotation speed, right speed, x speed(respective to drive mode)
           - mode: the mode of drive being used. Arcade, Tank, and Goblin respectively.
       note on "goblin mode":
           "goblin mode" is an attempt to immitate strafing seen in swerve
    */
    fun drive(arg1: Double, arg2: Double, mode: Constants.DriveMode) {
        when (mode) {
            Constants.DriveMode.ARCADE -> {
                drive.arcadeDrive(arg1, arg2)
            }

            Constants.DriveMode.TANK -> {
                drive.tankDrive(arg1, arg2)
            }
        }
    }

    fun driveArcadeConsumer(xspeed: DoubleSupplier, zrot: DoubleSupplier) {
        drive.arcadeDrive(xspeed.getAsDouble(), zrot.getAsDouble())
    }

    fun getPose(): Pose2d {
        return poseEstimator.getEstimatedPosition()
    }

    fun getWheelSpeeds(): DifferentialDriveWheelSpeeds {
        return DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity())
    }

    fun zeroGyro() {
        gyro.zeroYaw()
    }

    fun getYaw(): Double {
        return gyro.getYaw().toDouble() + startingPose.rotation.degrees
    }

    fun getPitch(): Double {
        return gyro.getPitch().toDouble()
    }

    /** Updates the field-relative position. */
    fun updateOdometry() {
        val visionResult: EstimatedRobotPose? = camera.getEstimatedPose(getPose())
        if (visionResult != null) {
            poseEstimator.addVisionMeasurement(visionResult.estimatedPose.toPose2d(), visionResult.timestampSeconds)
        }
        poseEstimator.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition())
    }

    /** Resets the drive encoders to currently read a position of 0. */
    fun resetEncoders() {

        leftEncoder.setPosition(0.0)

        rightEncoder.setPosition(0.0)
    }

    /**
     *
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    fun getAverageEncoderDistance(): Double =
        (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0

    /**
     *
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    fun setMaxOutput(maxOutput: Double) {
        drive.setMaxOutput(maxOutput)
    }

    /** Zeroes the robot's heading. */
    fun zeroHeading() {
        gyro.reset()
    }

    /**
     *
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    fun getHeading(): Double = gyro.getRotation2d().getDegrees()

    /**
     *
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    fun getTurnRate(): Double = -gyro.getRate()

    fun tankDriveVolts(leftVolts: Double, rightVolts: Double) {
        driveLeftFront.setVoltage(leftVolts)
        driveLeftBack.setVoltage(leftVolts)
        driveRightFront.setVoltage(rightVolts)
        driveRightFront.setVoltage(rightVolts)
        drive.feed()
    }
}
