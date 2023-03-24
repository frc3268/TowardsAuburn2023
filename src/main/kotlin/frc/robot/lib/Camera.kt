package frc.robot.lib

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.Constants
import java.io.IOException
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

class Camera {
    public val limelight: PhotonCamera = PhotonCamera("CCP BALOON CAMERA")
    public var frame: PhotonPipelineResult = PhotonPipelineResult()
    var poseEstimator: PhotonPoseEstimator? = null

    init {
        try {
            poseEstimator =
                PhotonPoseEstimator(
                    AprilTagFieldLayout(
                        Filesystem.getDeployDirectory().toString() +
                        "/2023-chargedup.json"
                    ),
                    PoseStrategy.MULTI_TAG_PNP,
                    limelight,
                    Transform3d()
                )
        } catch (e: IOException) {
            DriverStation.reportError("AprilTag: Failed to Load", e.getStackTrace())
            // !add some way to lock down apriltage features after this
        }
    }

    public fun getTankDirectionToTarget(
        target: PhotonTrackedTarget?,
        targetHeight: Double
    ): Constants.TankDirection? {
        return if (target != null) {
            val lengthForward = (targetHeight - Constants.Camera.camHeight) /
                Math.tan(Math.toRadians(target.pitch - Constants.Camera.cameraAngle))
            return Constants.TankDirection(
                lengthForward,
                target.yaw
            )
        } else null
    }

    public fun getTarget(bestTarget: Boolean, index: Int): PhotonTrackedTarget? {
        limelight.pipelineIndex = Constants.LimelightPipelineIndexes.reflectiveTape

        var output: PhotonTrackedTarget? =
            if (!frame.hasTargets()) null
            else if (bestTarget) frame.getBestTarget() else frame.targets[index]

        limelight.pipelineIndex = Constants.LimelightPipelineIndexes.aprilTag

        return output
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
