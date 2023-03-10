package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.Constants
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.PhotonPoseEstimator
import org.photonvision.EstimatedRobotPose
import java.io.IOException

class CameraSubsystem{
    val cam: PhotonCamera = PhotonCamera("CCP BALOON CAMERA")
    public var frame: PhotonPipelineResult = PhotonPipelineResult()
    var poseEstimator: PhotonPoseEstimator? = null
    

    init {
        try {
            poseEstimator =
                PhotonPoseEstimator(
                    AprilTagFieldLayout(
                        Filesystem.getDeployDirectory().toString()
                        + "/2023-chargedup.json"
                    ),
                    PoseStrategy.MULTI_TAG_PNP,
                    cam,
                    Transform3d()
                )
        } catch (e: IOException) {
            DriverStation.reportError("AprilTag: Failed to Load", e.getStackTrace())
            // !add some way to lock down apriltage features after this
        }
    }

    public fun getTranslationToTarget(
        target: PhotonTrackedTarget?,
        targetHeight: Double
    ): Translation2d? {
        if(target != null){
        var lengthForward = (targetHeight - Constants.Camera.camHeight) / Math.tan(target.pitch - Constants.Camera.cameraAngle)
        var lengthStafe = lengthForward / Math.tan(target.yaw)

            return Translation2d(lengthForward, lengthStafe)
        }
        return null
    }

    public fun getTarget(bestTarget: Boolean, index: Int): PhotonTrackedTarget? {
        cam.pipelineIndex = 0
        if (frame.hasTargets()) {
            cam.pipelineIndex = 1
            return null
        }
        if (bestTarget) {
            val target = frame.getBestTarget()
            cam.pipelineIndex = 1
            return target

        }
        val target = frame.targets[index]
        cam.pipelineIndex = 1
        return target
    }

    fun getEstimatedPose(prevPose: Pose2d): EstimatedRobotPose? {
        poseEstimator ?: return null
        poseEstimator?.setReferencePose(prevPose)
        return poseEstimator?.update()?.orElse(null)
    }

    fun resetPose(pose: Pose2d) {
        poseEstimator ?: return
        poseEstimator?.setReferencePose(pose)
    }
}
