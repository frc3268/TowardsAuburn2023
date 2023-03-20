package frc.robot.subsystems

import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.lib.Camera
import frc.robot.lib.units.*

class DriveSubsystem : SubsystemBase() {
    public val gyro: AHRS = AHRS(SPI.Port.kMXP)
    // Controllers
    private val driveLeftFront: CANSparkMax =
            CANSparkMax(Constants.Drive.leftFrontID, MotorType.kBrushless)
    private val driveLeftBack: CANSparkMax =
            CANSparkMax(Constants.Drive.leftBackID, MotorType.kBrushless)
    private val driveRightFront: CANSparkMax =
            CANSparkMax(Constants.Drive.rightFrontID, MotorType.kBrushless)
    private val driveRightBack: CANSparkMax =
            CANSparkMax(Constants.Drive.rightBackID, MotorType.kBrushless)

    //Encoders
    private val leftEncoder:RelativeEncoder = driveLeftFront.getEncoder()
    private val rightEncoder:RelativeEncoder = driveRightFront.getEncoder()

    // Groups
    private val driveLeft: MotorControllerGroup =
            MotorControllerGroup(driveLeftFront, driveLeftBack)
    private val driveRight: MotorControllerGroup =
            MotorControllerGroup(driveRightFront, driveRightBack)

    // Drive
    private val drive: DifferentialDrive = DifferentialDrive(driveLeft, driveRight)

    // PID
    // constants should be tuned per robot
    val linearP: Double = 0.6
    val linearD: Double = 0.0
    public val forwardController: PIDController = PIDController(linearP, 0.0, linearD)

    val angularP: Double = 0.3
    val angularD: Double = 0.0
    public val turnController = PIDController(angularP, 0.0, angularD)

    public val camera: Camera = Camera()

    //Odometry

    private val odometry:DifferentialDriveOdometry;

    init {
        gyro.calibrate()
        zeroGyro()
        odometry = DifferentialDriveOdometry(
            gyro.getRotation2d(), leftEncoder.getPosition(), leftEncoder.getPosition());
  
    }

    override fun periodic() {
        camera.frame = camera.limelight.getLatestResult()
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
    public fun drive(arg1: Double, arg2: Double, mode: Constants.DriveMode) {
        when (mode) {
            Constants.DriveMode.ARCADE -> {
                drive.arcadeDrive(arg1, arg2)
            }
            Constants.DriveMode.TANK -> {
                drive.tankDrive(arg1, arg2)
            }
            Constants.DriveMode.GOBLIN -> {
                val targetAngle = Math.atan(arg1 / arg2).rad.deg
                // if you're close to the direct forward or back, you dont deserve "goblin mode"
                val delta =
                        if (targetAngle > getYaw()) targetAngle - getYaw()
                        else getYaw() - targetAngle
                if ((Math.abs(delta - 180 ) > 22.5)) {
                    drive(arg1, arg2, Constants.DriveMode.ARCADE)
                } else {
                    while (delta > 5.0) {
                        // test this
                        drive.arcadeDrive(0.0, turnController.calculate(getYaw(), delta))
                    }
                    drive.arcadeDrive(Math.sqrt(Math.pow(arg1, 2.0) + Math.pow(arg2, 2.0)), 0.0)
                }
            }
        }
    }

    public fun zeroGyro() {
        gyro.zeroYaw()
    }

    public fun getYaw(): Double {
        return gyro.getYaw().toDouble() + Constants.Drive.startYaw
    }

    public fun getPitch(): Double {
        return gyro.getPitch().toDouble()
    }
      /** Updates the field-relative position. */
  public fun updateOdometry() {
    odometry.update(
        gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }
}
