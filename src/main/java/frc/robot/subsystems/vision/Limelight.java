// package frc.robot.subsystems.vision;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Limelight extends SubsystemBase {
//     NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
//     int targetID;
//     Transform3d cameraToTarget = new Transform3d();
//     private AprilTagFieldLayout aprilTagFieldLayout;

//     public Limelight() {
//         UsbCamera limelight = CameraServer.startAutomaticCapture(0);

//         limelight.setBrightness(50); //?
//         limelight.setExposureManual(15);

//         AprilTagFieldLayout layout;

//         try {
//             layout =
// AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
//             var alliance = DriverStation.getAlliance();
//         }
//     }
// }
