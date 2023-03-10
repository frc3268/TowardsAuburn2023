package frc.robot.subsystems

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import org.photonvision.EstimatedRobotPose

class DriveSubsystem : SubsystemBase() {
    public val swervePoseEstimator: SwerveDrivePoseEstimator

    public val swerveMods: List<SwerveModule> =
        Constants.Swerve.swerveMods.mapIndexed { i, swerveMod -> SwerveModule(i, swerveMod) }
    public val gyro: AHRS = AHRS(SPI.Port.kMXP)

    //PID Controllers

    // PID constants should be tuned per robot
    val linearP: Double = 0.6
    val linearD: Double = 0.0
    public val driveController: PIDController = PIDController(linearP, 0.0, linearD)

    //camera
    public val cam:Camera = Camera()



    init {
        gyro.calibrate()
        zeroGyro()
        swervePoseEstimator = SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), Pose2d(Translation2d(0.0,0.0), getYaw()))
        Timer.delay(1.0)
        resetModulesToAbsolute()
    }

    override fun periodic() {
        // This method will be called once per scheduler run
        swervePoseEstimator.update(getYaw(), getModulePositions()); 
        
        var visionResult : EstimatedRobotPose? = cam.getEstimatedPose(getPose())
        if(visionResult != null){
            swervePoseEstimator.addVisionMeasurement(visionResult.estimatedPose.toPose2d(), visionResult.timestampSeconds)
        }
        

        cam.frame = cam.limelight.getLatestResult()

        for(mod in swerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public fun drive(translation: Translation2d, rotation: Double, isOpenLoop: Boolean, fieldOriented: Boolean) {
        val swerveModuleStates: Array<SwerveModuleState> = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            if(fieldOriented) {
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getYaw()
                )
            } else {
                ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation
                )
            }
        )

        setModuleStates(swerveModuleStates, isOpenLoop)
    }

    public fun setModuleStates(desiredStates: Array<SwerveModuleState>, isOpenLoop: Boolean) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed)

        for (mod in swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop)
        }
    }

    public fun getPose(): Pose2d {
        return swervePoseEstimator.getEstimatedPosition()
    }

    public fun resetOdometry(pose: Pose2d) {
        swervePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose)
    }

    public fun getModuleStates(): Array<SwerveModuleState> {
        return swerveMods.map { it.getState() }
            .toTypedArray()
    }

    public fun getModulePositions(): Array<SwerveModulePosition> {
        return swerveMods.map { it.getPosition() }
            .toTypedArray()
    }

    public fun zeroGyro() {
        gyro.zeroYaw()
    }

    public fun getYaw(): Rotation2d {
        return Rotation2d.fromDegrees(gyro.getYaw().toDouble())
    }

    public fun getPitch(): Rotation2d {
        return Rotation2d.fromDegrees(gyro.getPitch().toDouble())
    }

    public fun resetModulesToAbsolute() {
        for (mod in swerveMods) {
            mod.resetToAbsolute()
        }
    }
}
