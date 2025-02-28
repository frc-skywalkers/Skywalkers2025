// package frc.robot.subsystems.vision;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import java.util.Optional;
// import org.photonvision.EstimatedRobotPose;
// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Transform3d;

// public class Camera {
//     private PhotonCamera camera;
//     private PhotonPoseEstimator poseEstimator;
//     private double lastEstTimestamp = 0;
//     public final Transform3d robotToCamera;

//     public Camera(String name, Transform3d robotToCamera) {
//         camera = new PhotonCamera(name);
//         this.robotToCamera = robotToCamera;

//         poseEstimator = new PhotonPoseEstimator(
//             AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
//             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//             robotToCamera);
//         poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//     }

//     public void updateEstimator() {
//         //all results since last called; call ONCE per loop
//         for (var result : camera.getAllUnreadResults()) {
//             if (result.multitagResult.isPresent()) { //case where multiple tags are detected
//                 var multitagResult = result.multitagResult.get();
//                 Transform3d fieldToCamera = multitagResult.estimatedPose.best;
//                 Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
//                 Pose3d estRobotPose = new Pose3d(fieldToRobot.getTranslation(),
// fieldToRobot.getRotation());
//                 return estRobotPose;
//             }
//         }
//     }
// }
