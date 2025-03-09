package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

  private final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  // Get alliance color
  DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);

  // Define reef AprilTag IDs for each alliance
  List<Integer> blueReefTags = List.of(17, 18, 19, 20, 21, 22); // Example IDs for Blue
  List<Integer> redReefTags = List.of(6, 7, 8, 9, 10, 11);

  // Select the correct array based on alliance color
  List<Integer> reefTags = (alliance == DriverStation.Alliance.Blue) ? blueReefTags : redReefTags;

  private Pose2d coral1Pose;
  private Pose2d coral2Pose;
  private Pose2d algeaPose;

  private int cachedTag = -1;

  // Slot Debug
  StructPublisher<Pose2d> coral1Publisher = NetworkTableInstance.getDefault()
      .getStructTopic("Coral 1", Pose2d.struct).publish();
  StructPublisher<Pose2d> coral2Publisher = NetworkTableInstance.getDefault()
      .getStructTopic("Coral 2", Pose2d.struct).publish();
  StructPublisher<Pose2d> algeaPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("Algea", Pose2d.struct).publish();

  StructPublisher<Pose2d> algaeOffsetPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("Algae Offset", Pose2d.struct).publish();

  public VisionSubsystem() {
    LimelightHelpers.setPipelineIndex(Vision.LIMELIGHT_3, 0);
  }

  @Override
  public void periodic() {
    int currentTag = getTargetId();

    // Check if the detected tag is a valid reef tag
    if (currentTag != -1) {
      if (!reefTags.contains(currentTag)) {
        return;
      }
    }

    cachedTag = currentTag; // Update cache
    updateSlotPositions(currentTag);
  }

  public PoseEstimate getPoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Vision.LIMELIGHT_3);
  }

  public int getTargetId() {
    return (int) LimelightHelpers.getFiducialID(Vision.LIMELIGHT_3);
    // return 21;
  }

  public Optional<Pose2d> getTagPose() {
    Optional<Pose2d> tagPose = field.getTagPose(getTargetId()).map(pose -> pose.toPose2d());
    return tagPose;
  }

  private void updateSlotPositions(int tagId) {
    Optional<Pose2d> tagPoseOptional = getTagPose();
    if (tagPoseOptional.isEmpty()) {
      coral1Pose = null;
      coral1Pose = null;
      algeaPose = null;

      coral1Publisher.set(coral1Pose);
      coral2Publisher.set(coral2Pose);
      algeaPublisher.set(algeaPose);

      return;
    }

    Pose2d tagPose = tagPoseOptional.get();

    Translation2d coral1Translation = new Translation2d(0.9, 0.2);
    Translation2d coral2Translation = new Translation2d(0.9, -0.2);
    Translation2d algeaTranslation = new Translation2d(0.9, 0);

    Transform2d coral1Transform = new Transform2d(coral1Translation, new Rotation2d(Math.PI));
    Transform2d coral2Transform = new Transform2d(coral2Translation, new Rotation2d(Math.PI));
    Transform2d algeaTransform = new Transform2d(algeaTranslation, new Rotation2d(Math.PI));

    coral1Pose = tagPose.transformBy(coral1Transform);
    coral2Pose = tagPose.transformBy(coral2Transform);
    algeaPose = tagPose.transformBy(algeaTransform);

    algaeOffsetPublisher.set(algeaPose.transformBy(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d())));

    // Advantagescope debug
    coral1Publisher.set(coral1Pose);
    coral2Publisher.set(coral2Pose);
    algeaPublisher.set(algeaPose);
  }

  public Pose2d getCoral1Pose() {
    return coral1Pose;
  }

  public Pose2d getCoral2Pose() {
    return coral2Pose;
  }

  public Pose2d getAlgeaPose() {
    return algeaPose;
  }
}
