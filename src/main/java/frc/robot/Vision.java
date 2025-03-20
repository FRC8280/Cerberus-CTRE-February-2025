/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot;

 import static frc.robot.Constants.Vision.*;
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Pose3d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Transform2d;
 import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.math.geometry.Translation3d;
 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.math.geometry.Translation3d;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import edu.wpi.first.wpilibj.smartdashboard.Field2d;
 import java.util.List;
 import java.util.Map;
 import java.util.Optional;
 import java.util.Queue;
 
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonTrackedTarget;
 //import org.photonvision.targeting.PhotonPipelineResult;
 import java.util.HashMap;
 import java.util.Map;
 
 import edu.wpi.first.wpilibj.DriverStation;
 import edu.wpi.first.wpilibj.DriverStation.Alliance;
 
 public class Vision {
     public final PhotonCamera camera;
     private final PhotonPoseEstimator photonEstimator;
     private Matrix<N3, N1> curStdDevs;
 
     // Simulation
     private PhotonCameraSim cameraSim;
     private VisionSystemSim visionSim;
 
     // Lookup table for angles
     private  Map<Integer, Double> angleLookupTable;
 
     private void populateAngleLookupTable() {
 
         angleLookupTable = new HashMap<Integer,Double>();
         // Populate the lookup table with values for 17-22 and 6-11
         angleLookupTable.put(17,60.0);
         angleLookupTable.put(18,0.0);
         angleLookupTable.put(19,-60.0);
         angleLookupTable.put(20,-120.0);
         angleLookupTable.put(21,180.0);
         angleLookupTable.put(22,120.0);
         angleLookupTable.put(6, 120.0);
         angleLookupTable.put(7, 180.0);
         angleLookupTable.put(8,-120.0 );
         angleLookupTable.put(9, -60.0);
         angleLookupTable.put(10, 0.0);
         angleLookupTable.put(11, 60.0 );
     }
 
     public double normalizeAngle(double angle) {
         // Normalize the angle to be within the range [0, 360)
         angle = angle % 360;
         if (angle < 0) {
             angle += 360;
         }
         return angle;
     }
 
     public double AngleDifference(double robotAngle, int tag)
     {
         double normalizeAngle = normalizeAngle(robotAngle);
         double normalizeTagAngle = normalizeAngle(getAngleForIndex(tag));
 
         return Math.abs(normalizeAngle - normalizeTagAngle);
   
     }
     public int CompareAngles(double robotAngle, double angle1, double angle2)
     {
         // Normalize the angles
         robotAngle = normalizeAngle(robotAngle);
         angle1 = normalizeAngle(angle1);
         angle2 = normalizeAngle(angle2);
 
         // Calculate the absolute differences
         double diff1 = Math.abs(robotAngle - angle1);
         double diff2 = Math.abs(robotAngle - angle2);
 
         // Compare the differences
         if (diff1 < diff2) {
             return -1;
         } else if (diff2 < diff1) {
             return 1;
         } else {
             return 0; // They match
         }
     }
 
     public double PercentageAngle(double robotAngle,int tag)
     {
         double normalizeAngle = normalizeAngle(robotAngle);
         double normalizeTagAngle = normalizeAngle(getAngleForIndex(tag));
 
         //Don't divide by 0
         if(normalizeAngle == 0)
             normalizeAngle = 0.1;
         if(normalizeTagAngle == 0.0)
             normalizeTagAngle = 0.1;
         
         return normalizeAngle/normalizeTagAngle;
     }
 
     public double getAngleForIndex(int index) {
         return angleLookupTable.getOrDefault(index, 0.0); // Return 0.0 if index is not found
     }
 
     public static double angleBetweenPoses(Pose2d pose1, Pose2d pose2) {
         // Calculate the direction vectors
         Translation2d direction1 = new Translation2d(Math.cos(pose1.getRotation().getRadians()),
                 Math.sin(pose1.getRotation().getRadians()));
         Translation2d direction2 = new Translation2d(Math.cos(pose2.getRotation().getRadians()),
                 Math.sin(pose2.getRotation().getRadians()));
 
         // Calculate the dot product of the direction vectors
         double dotProduct = direction1.getX() * direction2.getX() + direction1.getY() * direction2.getY();
 
         // Calculate the magnitudes of the direction vectors
         double magnitude1 = direction1.getNorm();
         double magnitude2 = direction2.getNorm();
 
         // Calculate the angle between the vectors using the dot product formula
         double angle = Math.acos(dotProduct / (magnitude1 * magnitude2));
 
         // Convert the angle to degrees
         return Math.toDegrees(angle);
     }
 
     public Pose2d addXYToPose(Pose2d originalPose, double x, double y, double angle) {
         // Create a Transform2d object with the desired translation
         Transform2d translationTransform = new Transform2d(new Translation2d(x, y), Rotation2d.fromDegrees(angle));
 
         // Apply the transformation to the original pose
         Pose2d newPose = originalPose.plus(translationTransform);
 
         return newPose;
     }
 
     public double calculateDistanceBetweenPoseAndTransform(Pose2d pose2d, Transform3d transform3d) {
         // Convert Pose2d to Pose3d by adding a zero z-coordinate
         Pose3d pose2dAsPose3d = new Pose3d(pose2d);
         // Apply the transformation to the Pose3d
         Pose3d transformedPose = pose2dAsPose3d.transformBy(transform3d);
 
         // Get the translation components of the poses
         Translation3d translation1 = pose2dAsPose3d.getTranslation();
         Translation3d translation2 = transformedPose.getTranslation();
 
         // Calculate the Euclidean distance between the translations
         double distance = translation1.getDistance(translation2);
 
         return distance;
     }
     
     public boolean isIntegerInList(int number, List<Integer> list) {
         return list.contains(number);
     }
 
     public boolean CheckValidAprilTag(int aprilTag) {
         // List of valid AprilTag IDs
         Optional<Alliance> ally = DriverStation.getAlliance();
         if (ally.get() == Alliance.Red) {
             if (aprilTag >= 6 && aprilTag <= 11) {
                 return true;
             }
         } else if (ally.get() == Alliance.Blue) {
             if (aprilTag >= 17 && aprilTag <= 22) {
                 return true;
             }
         }
         return false;
     }
 
     public ReefTargets CalculateAutoReefTarget(int aprilTag) {
         
         Optional<Alliance> ally = DriverStation.getAlliance();
         ReefTargets returnTargets = new ReefTargets();
         if (ally.get() == Alliance.Red) {
             switch (aprilTag) {
                 case 6:
                     returnTargets.leftTarget = new Pose2d(13.524, 2.66, Rotation2d.fromDegrees(119.78));
                     returnTargets.rightTarget = new Pose2d(13.799, 2.927, Rotation2d.fromDegrees(119.9));
                     returnTargets.centerTarget = new Pose2d(12.441, 5.18, Rotation2d.fromDegrees(199.8));
                     break;
                 case 7:
                     returnTargets.leftTarget = new Pose2d(14.437, 3.742, Rotation2d.fromDegrees(179.311));
                     returnTargets.rightTarget = new Pose2d(14.437, 4.06, Rotation2d.fromDegrees(178.02));
                     returnTargets.centerTarget = new Pose2d(14.437, 3.996, Rotation2d.fromDegrees(177.417));
                     break;
                 case 8:
                     returnTargets.leftTarget = new Pose2d(13.955, 5.036, Rotation2d.fromDegrees(-121.99));
                     returnTargets.rightTarget = new Pose2d(13.601, 5.192, Rotation2d.fromDegrees(-122.05));
 
                     returnTargets.centerTarget = new Pose2d(13.89, 5.151, Rotation2d.fromDegrees(-121.185));
                     break;
                 case 9:
                     returnTargets.leftTarget = new Pose2d(12.657, 5.2081, Rotation2d.fromDegrees(-62.49));
                     returnTargets.rightTarget = new Pose2d(12.282, 5.06103, Rotation2d.fromDegrees(-61.45));
                     
                     returnTargets.centerTarget = new Pose2d(12.450, 5.199, Rotation2d.fromDegrees(-62.135));
                     break;
                 case 10:
                     returnTargets.leftTarget = new Pose2d(11.6626, 4.2804, Rotation2d.fromDegrees(-2.8));
                     returnTargets.rightTarget = new Pose2d(11.642077225482328, 3.919858379891, Rotation2d.fromDegrees(-.078));
                     
                     returnTargets.centerTarget = new Pose2d(11.728, 4.055, Rotation2d.fromDegrees(0.099));
                     break;
                 case 11:
                     returnTargets.leftTarget = new Pose2d(12.110, 2.8954, Rotation2d.fromDegrees(57.55));
                     returnTargets.rightTarget = new Pose2d(12.4160, 2.802, Rotation2d.fromDegrees(58.64));
                     
                     returnTargets.centerTarget = new Pose2d(12.422, 2.88, Rotation2d.fromDegrees(59.365));
                     break;
                 default:
                     returnTargets.leftTarget = null;
                     returnTargets.rightTarget = null;
                     returnTargets.centerTarget = null;
                     break;
             }
         }
 
         else if (ally.get() == Alliance.Blue) {
             switch (aprilTag) {
                 case 17:
                     returnTargets.leftTarget = new Pose2d(3.624, 2.9964, Rotation2d.fromDegrees(60.35/*53.6*/));
                     returnTargets.rightTarget = new Pose2d(3.950, 2.793, Rotation2d.fromDegrees(61.35));
                     
                     returnTargets.centerTarget = new Pose2d(3.810, 2.899, Rotation2d.fromDegrees(60.715));
                     break;
                 case 18:
                     returnTargets.leftTarget = new Pose2d(3.02, 4.25, Rotation2d.fromDegrees(2.4));
                     returnTargets.rightTarget = new Pose2d(3.1, 3.9350, Rotation2d.fromDegrees(2.0));
                     
                     returnTargets.centerTarget = new Pose2d(3.175, 4.112, Rotation2d.fromDegrees(-2.96));
                     break;
                 case 19:
                     returnTargets.leftTarget = new Pose2d(3.99474, 5.34948, Rotation2d.fromDegrees(-57.4));
                     returnTargets.rightTarget = new Pose2d(3.6625, 5.2106, Rotation2d.fromDegrees(-56.982));
                     
                     returnTargets.centerTarget = new Pose2d(3.737354424938496, 4.940004156734, Rotation2d.fromDegrees(-61.2695));
                     break;
                 case 20:
                     returnTargets.leftTarget = new Pose2d(5.37180, 5.095107, Rotation2d.fromDegrees(-121.58));
                     returnTargets.rightTarget = new Pose2d(5.12041, 5.28087, Rotation2d.fromDegrees(-119.38));
                     
                     returnTargets.centerTarget = new Pose2d(5.225, 5.122, Rotation2d.fromDegrees(-122.065));
                     break;
                 case 21:
                     returnTargets.leftTarget = new Pose2d(5.87825, 3.762089, Rotation2d.fromDegrees(178.718));
                     returnTargets.rightTarget = new Pose2d(5.867954, 4.10801, Rotation2d.fromDegrees(-177.0344));
 
                     returnTargets.centerTarget = new Pose2d(5.8, 3.958, Rotation2d.fromDegrees(-178.19));
                     break;
                 case 22:
                     returnTargets.leftTarget = new Pose2d(4.875, 2.79, Rotation2d.fromDegrees(115.406));
                     returnTargets.rightTarget = new Pose2d(5.260, 2.90, Rotation2d.fromDegrees(121.6));
                     
                     returnTargets.centerTarget = new Pose2d(4.6141, 2.80, Rotation2d.fromDegrees(117.088));
                     break;
                 default:
                     returnTargets.leftTarget = null;
                     returnTargets.rightTarget = null;
                     returnTargets.centerTarget = null;
                     break;
             }
         }   
 
         return returnTargets;
     }
 
     public Vision(int index) {
 
         populateAngleLookupTable();
         camera = new PhotonCamera(Constants.Vision.CamNames[index]);
 
         photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                 cameraTransforms[index]);
         photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
 
         // ----- Simulation
         if (Robot.isSimulation()) {
             // Create the vision system simulation which handles cameras and targets on the
             // field.
             visionSim = new VisionSystemSim("main");
             // Add all the AprilTags inside the tag layout as visible targets to this
             // simulated field.
             visionSim.addAprilTags(kTagLayout);
             // Create simulated camera properties. These can be set to mimic your actual
             // camera.
             var cameraProp = new SimCameraProperties();
             cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
             cameraProp.setCalibError(0.35, 0.10);
             cameraProp.setFPS(15);
             cameraProp.setAvgLatencyMs(50);
             cameraProp.setLatencyStdDevMs(15);
             // Create a PhotonCameraSim which will update the linked PhotonCamera's values
             // with visible
             // targets.
             cameraSim = new PhotonCameraSim(camera, cameraProp);
             // Add the simulated camera to view the targets on this simulated field.
             visionSim.addCamera(cameraSim, cameraTransforms[index]);
 
             cameraSim.enableDrawWireframe(true);
         }
     }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be
      * empty. This should
      * only be called once per loop.
      *
      * <p>
      * Also includes updates for the standard deviations, which can (optionally) be
      * retrieved with
      * {@link getEstimationStdDevs}
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
      *         timestamp, and targets
      *         used for estimation.
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         for (var change : camera.getAllUnreadResults()) {
             visionEst = photonEstimator.update(change);
             updateEstimationStdDevs(visionEst, change.getTargets());
 
             if (Robot.isSimulation()) {
                 visionEst.ifPresentOrElse(
                         est -> getSimDebugField()
                                 .getObject("VisionEstimation")
                                 .setPose(est.estimatedPose.toPose2d()),
                         () -> {
                             getSimDebugField().getObject("VisionEstimation").setPoses();
                         });
             }
         }
         return visionEst;
     }
 
     /**
      * Calculates new standard deviations This algorithm is a heuristic that creates
      * dynamic standard
      * deviations based on number of tags, estimation strategy, and distance from
      * the tags.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      * @param targets       All targets in this camera frame
      */
     private void updateEstimationStdDevs(
             Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             curStdDevs = kSingleTagStdDevs;
 
         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = kSingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
 
             // Precalculation - see how many tags we found, and calculate an
             // average-distance metric
             for (var tgt : targets) {
                 var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty())
                     continue;
                 numTags++;
                 avgDist += tagPose
                         .get()
                         .toPose2d()
                         .getTranslation()
                         .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
             }
 
             if (numTags == 0) {
                 // No tags visible. Default to single-tag std devs
                 curStdDevs = kSingleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1)
                     estStdDevs = kMultiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else
                     estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                 curStdDevs = estStdDevs;
             }
         }
     }
 
     /**
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getEstimatedGlobalPose()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
      * SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public Matrix<N3, N1> getEstimationStdDevs() {
         return curStdDevs;
     }
 
     // ----- Simulation
 
     public void simulationPeriodic(Pose2d robotSimPose) {
         visionSim.update(robotSimPose);
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (Robot.isSimulation())
             visionSim.resetRobotPose(pose);
     }
 
     /** A Field2d for visualizing our robot and objects on the field. */
     public Field2d getSimDebugField() {
         if (!Robot.isSimulation())
             return null;
         return visionSim.getDebugField();
     }
 }