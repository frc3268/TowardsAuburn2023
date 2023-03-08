package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.geometry.Translation2d
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.PhotonCamera
import frc.robot.Constants

class CameraSubsystem : SubsystemBase() {
    val cam: PhotonCamera = PhotonCamera("CCP BALOON CAMERA")
    public var frame: PhotonPipelineResult = PhotonPipelineResult()

    public fun getTranslationToTarget(target:PhotonTrackedTarget, targetHeightI:Double):Translation2d{
        var lengthForward = (targetHeightI - Constants.Camera.camHeightI) / Math.tan(target.pitch)
        var lengthStafe = lengthForward/Math.tan(target.yaw)

        return Translation2d(lengthForward, lengthStafe)
    }

    
    override fun periodic() {
        // This method will be called once per scheduler run
        frame = cam.getLatestResult()
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
