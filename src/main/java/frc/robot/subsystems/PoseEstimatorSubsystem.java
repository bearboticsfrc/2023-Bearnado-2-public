package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.DriveConstants;
import frc.constants.VisionConstants;
import frc.robot.fms.AllianceColor;
import frc.robot.fms.AllianceReadyListener;
import frc.robot.location.FieldPositions;
import frc.robot.subsystems.vision.EstimationRunnable;
import frc.robot.subsystems.vision.RobotCamera;
import frc.robot.subsystems.vision.StringFormatting;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PoseEstimatorSubsystem extends SubsystemBase implements AllianceReadyListener {

  private final SwerveDrivePoseEstimator poseEstimator;
  private final DriveSubsystem driveSubsystem;

  private RobotCamera[] robotCameras = {
    // new RobotCamera("CenterCam","Arducam_OV2311_USB_Camera",ROBOT_TO_MIDDLE_CAMERA),
    new RobotCamera("RightCam", "Arducam_OV9281_USB_Camera", VisionConstants.ROBOT_TO_RIGHT_CAMERA),
    new RobotCamera("LeftCam", "OV9281_2", VisionConstants.ROBOT_TO_LEFT_CAMERA),
  };

  private final Field2d field2d = new Field2d();

  private final DataLog dataLog;
  private final DoubleLogEntry poseXLog;
  private final DoubleLogEntry poseYLog;
  private final DoubleLogEntry poseAngleDegreesLog;
  private final StringLogEntry poseLog;

  private AprilTagFieldLayout layout;

  private List<Notifier> notifiers = new ArrayList<Notifier>();
  private List<EstimationRunnable> estimationRunnables = new ArrayList<EstimationRunnable>();

  public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem, FieldPositions fieldPositions) {
    this.driveSubsystem = driveSubsystem;

    layout = fieldPositions.getLayout();

    AllianceColor.addListener(this);

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            driveSubsystem.getHeading(),
            driveSubsystem.getModulePositions(),
            getInitialPose2d(),
            VisionConstants.STATE_STD_DEVS,
            VisionConstants.VISION_STD_DEVS);

    for (RobotCamera robotCamera : robotCameras) {
      PhotonCamera photonCamera = new PhotonCamera(robotCamera.getCameraName());

      EstimationRunnable estimatorRunnable =
          new EstimationRunnable(
              robotCamera.getNiceName(), photonCamera, robotCamera.getRobotToCameraTransform());
      estimationRunnables.add(estimatorRunnable);
      Notifier notifier = new Notifier(estimatorRunnable);
      notifiers.add(notifier);

      // Start PhotonVision thread
      notifier.setName(robotCamera.getNiceName());
      notifier.startPeriodic(0.02);
    }

    tab.addString("Pose", () -> StringFormatting.poseToString(getPose()))
        .withPosition(0, 0)
        .withSize(2, 1);
    tab.addString("Drive Pose", () -> StringFormatting.poseToString(driveSubsystem.getPose()))
        .withPosition(0, 1)
        .withSize(2, 1);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4).withWidget(BuiltInWidgets.kField);

    dataLog = DataLogManager.getLog();
    poseLog = new StringLogEntry(dataLog, "/vision/pose");
    poseXLog = new DoubleLogEntry(dataLog, "/vision/pose/x");
    poseYLog = new DoubleLogEntry(dataLog, "/vision/pose/y");
    poseAngleDegreesLog = new DoubleLogEntry(dataLog, "/vision/pose/angle");
  }

  private void logPose(Pose2d pose) {
    poseLog.append(StringFormatting.poseToString(pose));
    poseXLog.append(pose.getX());
    poseYLog.append(pose.getY());
    poseAngleDegreesLog.append(pose.getRotation().getDegrees());
  }

  public void updateAllianceColor(Alliance alliance) {
    // should this also re-initialize the pose estimators ?
    layout.setOrigin(
        alliance == Alliance.Blue
            ? OriginPosition.kBlueAllianceWallRightSide
            : OriginPosition.kRedAllianceWallRightSide);
  }

  private Pose2d getInitialPose2d() {
    return new Pose2d();
  }

  private void logTarget(PhotonTrackedTarget target) {
    DoubleLogEntry ambiguityLogEntry =
        new DoubleLogEntry(dataLog, "/pose/ambiguity" + target.getFiducialId());
    ambiguityLogEntry.append(target.getPoseAmbiguity());
    DoubleLogEntry transformXLogEntry =
        new DoubleLogEntry(dataLog, "/pose/X" + target.getFiducialId());
    transformXLogEntry.append(target.getBestCameraToTarget().getX());
    DoubleLogEntry transformYLogEntry =
        new DoubleLogEntry(dataLog, "/pose/Y" + target.getFiducialId());
    transformYLogEntry.append(target.getBestCameraToTarget().getY());
  }

  private boolean hasTargets;

  public boolean hasTargets() {
    return hasTargets;
  }

  private int bestFiducialId;

  public int bestFiducialId() {
    return bestFiducialId;
  }

  @Override
  public void periodic() {
    updateOdometry();
    updateVisionMeasurement();
    field2d.setRobotPose(getPose());
  }

  public void updateOdometry() {
    poseEstimator.update(driveSubsystem.getHeading(), driveSubsystem.getModulePositions());
  }

  public void updateVisionMeasurement() {
    for (EstimationRunnable estimationRunnable : estimationRunnables) {
      estimatorChecker(estimationRunnable);
    }
    /*
    for (PhotonEstimator photonEstimator : photonPoseEstimators) {

      timer.reset();
      timer.start();
      // Comment out the experiment to transform the robot camera translation
      // PhotonPipelineResult cameraResult = photonEstimator.camera.getLatestResult();
      // if (cameraResult == null || !cameraResult.hasTargets()) {
      //  continue;
      // }
      // Optional<EstimatedRobotPose> result =
      //     photonEstimator.estimator.update(normalizeAllTargets(cameraResult));
      Optional<EstimatedRobotPose> result = photonEstimator.estimator.update();
      timer.stop();
      // DataLogManager.log("photon update [" + estimator.getFirst() + "] took " + timer.get());
      if (result.isEmpty()) {
        continue;
      }
      EstimatedRobotPose robotPose = result.get();

      if (!targetsAreLowAmbiguity(robotPose.targetsUsed)) {
        continue;
      }
      Pose2d visionPose = robotPose.estimatedPose.toPose2d();
      Pose2d adjustedPose =
          new Pose2d(visionPose.getX(), visionPose.getY(), driveSubsystem.getHeading());
        DataLogManager.log(
                "New vision based pose["
                    + photonEstimator.name
                    + "]: "
                    + StringFormatting.estimatedRobotPoseToString(robotPose));


      // use the IMU for rotation instead of april tag pose
      poseEstimator.addVisionMeasurement(
          adjustedPose, robotPose.timestampSeconds, confidenceCalculator(robotPose));
    }
    */
  }

  /** Test routine to calculate the robot to target translation */
  private String getRobotToTarget(EstimatedRobotPose robotPose) {
    Transform3d CAMERA_TO_ROBOT =
        new Transform3d(
            // camera is .0717296 meters back, -0.2270252 left, 1.2596368 up
            // new Translation3d(0.0/* .0717296*/, 0.0,1.2596368),
            new Translation3d(0.0 /* .0717296*/, 0.0, 0.0),
            new Rotation3d(0, Math.toRadians(-19.0), 0));

    List<PhotonTrackedTarget> tags = robotPose.targetsUsed;
    Transform3d cameraToTarget = null;

    for (PhotonTrackedTarget target : tags) {
      cameraToTarget = target.getBestCameraToTarget();
    }
    Transform3d robotToTarget = cameraToTarget.plus(CAMERA_TO_ROBOT);
    // DataLogManager.log("Camera_to_target:" + StringFormatting.transformToString(cameraToTarget));
    // DataLogManager.log("Robot_to_target:" + StringFormatting.transformToString(robotToTarget));
    return StringFormatting.transformToString(robotToTarget);
  }

  private boolean targetsAreLowAmbiguity(List<PhotonTrackedTarget> tags) {
    for (PhotonTrackedTarget target : tags) {
      if (target.getPoseAmbiguity() > 0.20) return false;
    }
    return true;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
        driveSubsystem.getHeading(), driveSubsystem.getModulePositions(), newPose);
  }

  public String getPoseString() {
    return getPoseString(getPose());
  }

  public String getPoseString(Pose2d pose) {
    return String.format(
        "(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  public static final int RED_DOUBLE_SUBSTATION_FIDUCIAL_ID = 5;
  public static final int BLUE_DOUBLE_SUBSTATION_FIDUCIAL_ID = 4;

  public Pose2d getSubstationPose() {
    int doubleStationFiducialId = BLUE_DOUBLE_SUBSTATION_FIDUCIAL_ID;
    if (AllianceColor.alliance == Alliance.Red) {
      doubleStationFiducialId = RED_DOUBLE_SUBSTATION_FIDUCIAL_ID;
    }
    return layout.getTagPose(doubleStationFiducialId).get().toPose2d();
  }

  public double getDistanceToSubstationWall() {
    Transform2d transform = new Transform2d(getPose(), getSubstationPose());
    return transform.getX() - VisionConstants.HALF_ROBOT_LENGTH;
  }

  public double getLateralToSubstationWall() {
    Transform2d transform = new Transform2d(getPose(), getSubstationPose());
    return transform.getY();
  }

  private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
    double smallestDistance = Double.POSITIVE_INFINITY;
    for (var target : estimation.targetsUsed) {
      Transform3d t3d = target.getBestCameraToTarget();
      double distance =
          Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      if (distance < smallestDistance) {
        smallestDistance = distance;
      }
    }
    double poseAmbiguityFactor =
        estimation.targetsUsed.size() != 1
            ? 1
            : Math.max(
                1,
                (estimation.targetsUsed.get(0).getPoseAmbiguity()
                        + VisionConstants.POSE_AMBIGUITY_SHIFTER
                            * VisionConstants.POSE_AMBIGUITY_MULTIPLIER)
                    / (1
                        + ((estimation.targetsUsed.size() - 1)
                            * VisionConstants.TAG_PRESENCE_WEIGHT)));

    double confidenceMultiplier =
        Math.max(
            1,
            (Math.max(
                    1,
                    Math.max(0, smallestDistance - VisionConstants.NOISY_DISTANCE_METERS)
                        * VisionConstants.DISTANCE_WEIGHT)
                * poseAmbiguityFactor
                / (1
                    + ((estimation.targetsUsed.size() - 1)
                        * VisionConstants.TAG_PRESENCE_WEIGHT))));

    return VisionConstants.VISION_MEASUREMENT_STD_DEVS.times(confidenceMultiplier);
  }

  public void estimatorChecker(EstimationRunnable estamator) {
    EstimatedRobotPose robotPose = estamator.grabLatestEstimatedPose();
    if (robotPose != null) {
      if (!targetsAreLowAmbiguity(robotPose.targetsUsed)) {
        return;
      }
      Pose2d visionPose = robotPose.estimatedPose.toPose2d();
      Pose2d adjustedPose =
          new Pose2d(visionPose.getX(), visionPose.getY(), driveSubsystem.getHeading());

      logPose(adjustedPose);
      poseEstimator.addVisionMeasurement(
          adjustedPose, robotPose.timestampSeconds, confidenceCalculator(robotPose));
    }
  }
}
