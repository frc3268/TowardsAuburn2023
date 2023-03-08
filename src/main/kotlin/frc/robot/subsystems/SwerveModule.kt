package frc.robot.subsystems

import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.ctre.phoenix.sensors.CANCoderConfiguration
import com.ctre.phoenix.sensors.CANCoderStatusFrame
import com.ctre.phoenix.sensors.SensorInitializationStrategy
import com.ctre.phoenix.sensors.SensorTimeBase
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.kinematics.SwerveModulePosition
import frc.robot.Constants
import frc.robot.units.*

/** Add your docs here. */
class SwerveModule(moduleNumber: Int, moduleConstants: Constants.SwerveDriveModuleConstants) {
    public val moduleNumber: Int = moduleNumber
    private var lastangle: Rotation2d = Rotation2d.fromDegrees(0.deg)
    private val angleOffset: Rotation2d = moduleConstants.angleOffset

    private val angleMotor: CANSparkMax =
        CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless)
    private val driveMotor: CANSparkMax =
        CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless)

    private val driveEncoder: RelativeEncoder = driveMotor.getEncoder()
    private val integratedAngleEncoder: RelativeEncoder = angleMotor.getEncoder()
    private val angleEncoder: CANCoder = CANCoder(moduleConstants.canCoderID)

    private val driveController: SparkMaxPIDController = driveMotor.getPIDController()
    private val angleController: SparkMaxPIDController = angleMotor.getPIDController()

    private val canCoderConfig: CANCoderConfiguration = CANCoderConfiguration()

    private val feedforward: SimpleMotorFeedforward =
        SimpleMotorFeedforward(
            Constants.Swerve.driveKS,
            Constants.Swerve.driveKV,
            Constants.Swerve.driveKA
        )

    init {
        configDriveMotor()
        configAngleMotor()
        configAngleEncoder()
        lastangle = getState().angle
    }

    public fun setDesiredState(desiredState: SwerveModuleState, isOpenLoop: Boolean) {
        var dState: SwerveModuleState = optimize(desiredState, getState().angle)
        if(isOpenLoop) {
            setAngle(dState)
            //percentwanted * max = speedwanted
            driveMotor.set(dState.speedMetersPerSecond / Constants.Swerve.maxSpeed)
            return
        }
        setAngle(dState)
        setSpeed(dState)
    }
    /*
    The following three methods(configAngleEncoder, configDriveMotor, configAngleMotor) are used to set the config params of the motors and encoders.
    They are required for proper operation of the swerve drive system
     */
    fun configAngleEncoder() {
        canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360
        canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition
        canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond
        canCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert
        angleEncoder.configFactoryDefault()
        angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100)
        angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100)
        angleEncoder.configAllSettings(canCoderConfig)
    }

    fun configDriveMotor() {
        driveMotor.restoreFactoryDefaults()
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20)
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20)
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 50)
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit)
        driveMotor.setInverted(Constants.Swerve.driveInvert)
        driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode)
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor)
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor)
        driveController.setP(Constants.Swerve.angleKP)
        driveController.setI(Constants.Swerve.angleKI)
        driveController.setD(Constants.Swerve.angleKD)
        driveController.setFF(Constants.Swerve.angleKFF)
        driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp)
        driveMotor.burnFlash()
        driveEncoder.setPosition(0.0)
    }

    fun configAngleMotor() {
        angleMotor.restoreFactoryDefaults()
        angleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 500)
        angleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20)
        angleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 500)
        angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
        angleMotor.setInverted(Constants.Swerve.angleInvert)
        angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode)
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor)
        angleController.setP(Constants.Swerve.angleKP)
        angleController.setI(Constants.Swerve.angleKI)
        angleController.setD(Constants.Swerve.angleKD)
        angleController.setFF(Constants.Swerve.angleKFF)
        angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp)
        angleMotor.burnFlash()
        resetToAbsolute()
    }

    //setSpeed(): sets the speed of the drive motor of a swerve module based on a desiredState arguement
    fun setSpeed(desiredState: SwerveModuleState) {
        driveController.setReference(
            desiredState.speedMetersPerSecond,
            CANSparkMax.ControlType.kVelocity,
            0,
            feedforward.calculate(desiredState.speedMetersPerSecond)
        )
    }

    //setAngle(): sets the speed of the angle motor of a swerve module based on a desiredState arguement
    fun setAngle(desiredState: SwerveModuleState) {
        var angle: Rotation2d
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        angle = if(Math.abs(desiredState.speedMetersPerSecond) <= Constants.Swerve.maxSpeed * 0.01) {
            lastangle
        } else {
            desiredState.angle
        }

        angleController.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition)
        lastangle = angle
    }

    fun resetToAbsolute() {
        integratedAngleEncoder.setPosition(getCanCoder().getDegrees() - angleOffset.getDegrees())
    }

    fun getAngle(): Rotation2d {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition())
    }

    public fun getCanCoder(): Rotation2d {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition())
    }

    public fun getState(): SwerveModuleState {
        return SwerveModuleState(driveEncoder.getVelocity(), getAngle())
    }

    public fun getPosition(): SwerveModulePosition{
        return SwerveModulePosition(driveEncoder.getPosition(), getAngle())
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include placing
     * in appropriate scope for CTRE and REV onboard control as both controllers as of writing don't
     * have support for continuous input.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    fun optimize(desiredState: SwerveModuleState, currentAngle: Rotation2d): SwerveModuleState {
        var targetAngle: Double =
            placeInAppropriate0To360Scope(
                currentAngle.getDegrees(),
                desiredState.angle.getDegrees()
            )
        var targetSpeed: Double = desiredState.speedMetersPerSecond
        var delta: Double = targetAngle - currentAngle.getDegrees()
        if(Math.abs(delta) > 90.deg) {
            targetSpeed *= -1
            targetAngle += if(delta > 90.deg) { -180.deg } else { +180.deg }
        }
        return SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle))
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    fun placeInAppropriate0To360Scope(scopeReference: Double, newAngle: Double): Double {
        var lowerBound: Double
        var upperBound: Double
        var nAngle: Double = newAngle
        var lowerOffset: Double = scopeReference % 360.deg

        if (lowerOffset >= 0.deg) {
            lowerBound = scopeReference - lowerOffset
            upperBound = scopeReference + (360.deg - lowerOffset)
        } else {
            upperBound = scopeReference - lowerOffset
            lowerBound = scopeReference - (360.deg + lowerOffset)
        }

        while (newAngle < lowerBound) {
            nAngle += 360.deg
        }
        while (newAngle > upperBound) {
            nAngle -= 360.deg
        }

        return nAngle + when {
            newAngle - scopeReference > +180.deg -> -360.deg
            newAngle - scopeReference < -180.deg -> +360.deg
            else -> 0.deg
        }
    }
}
