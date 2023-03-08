package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

class CameraSubsystem : SubsystemBase() {
    val cam: PhotonCamera = PhotonCamera("CCP BALOON CAMERA")
    public var frame: PhotonPipelineResult = PhotonPipelineResult()

    public fun getTranslationToTarget(
            target: PhotonTrackedTarget,
            targetHeightI: Double
    ): Translation2d {
        var lengthForward = (targetHeightI - Constants.Camera.camHeightI) / Math.tan(target.pitch - Constants.Camera.cameraAngleD)
        var lengthStafe = lengthForward / Math.tan(target.yaw)

        return Translation2d(lengthForward, lengthStafe)
    }

    public fun getTarget(bestTarget: Boolean, index: Int): PhotonTrackedTarget? {
        if (frame.hasTargets()) {
            return null
        }
        if (bestTarget) {
            return frame.getBestTarget()
        }
        return frame.targets[index]
    }

    override fun periodic() {
        // This method will be called once per scheduler run
        frame = cam.getLatestResult()
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
