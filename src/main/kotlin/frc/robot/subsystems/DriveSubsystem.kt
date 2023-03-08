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
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

class DriveSubsystem : SubsystemBase() {
    public val swerveOdom: SwerveDriveOdometry

    public val swerveMods: Array<SwerveModule> =
        arrayOf(
            SwerveModule(0, Constants.Swerve.mod0),
            SwerveModule(1, Constants.Swerve.mod1),
            SwerveModule(2, Constants.Swerve.mod2),
            SwerveModule(3, Constants.Swerve.mod3)
        )
    public val gyro: AHRS = AHRS(SPI.Port.kMXP)

    init {
        gyro.calibrate()
        zeroGyro()
        swerveOdom = SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions())
        Timer.delay(1.0)
        resetModulesToAbsolute()
    }

    override fun periodic() {
        // This method will be called once per scheduler run
        swerveOdom.update(getYaw(), getModulePositions());  

        for(mod in swerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public fun drive(translation: Translation2d, rotation: Double, isOpenLoop:Boolean, fieldOriented:Boolean) {
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

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed)
        for (mod in swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop)
        }
    }

    public fun setModuleStates(desiredStates: Array<SwerveModuleState>, isOpenLoop: Boolean) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed)

        for (mod in swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop)
        }
    }

    public fun getPose(): Pose2d {
        return swerveOdom.getPoseMeters()
    }

    public fun resetOdometry(pose: Pose2d) {
        swerveOdom.resetPosition(getYaw(), getModulePositions(), pose)
    }

    public fun getModuleStates(): Array<SwerveModuleState> {
        return arrayOf(
            swerveMods[0].getState(),
            swerveMods[1].getState(),
            swerveMods[2].getState(),
            swerveMods[3].getState()
        )
    }

    public fun getModulePositions(): Array<SwerveModulePosition> {
        return arrayOf(
            swerveMods[0].getPosition(),
            swerveMods[1].getPosition(),
            swerveMods[2].getPosition(),
            swerveMods[3].getPosition()
        )
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
