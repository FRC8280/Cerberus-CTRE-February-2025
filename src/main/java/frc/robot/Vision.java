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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Vision {
    public final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

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
        // aprilTag = Robot.aprilTagId;

        int trimX = 0;
        int trimY = 0;
        double trimTheta = 0.0;

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
                    returnTargets.leftTarget = new Pose2d(14.051, 5.094, Rotation2d.fromDegrees(-121.99));
                    returnTargets.rightTarget = new Pose2d(13.74, 5.191, Rotation2d.fromDegrees(-122.05));
                    returnTargets.centerTarget = new Pose2d(13.89, 5.151, Rotation2d.fromDegrees(-121.185));
                    break;
                case 9:
                    returnTargets.leftTarget = new Pose2d(12.208, 5.3488, Rotation2d.fromDegrees(-62.82));
                    returnTargets.rightTarget = new Pose2d(12.605, 5.1598, Rotation2d.fromDegrees(-61.45));
                    returnTargets.centerTarget = new Pose2d(12.450, 5.199, Rotation2d.fromDegrees(-62.135));
                    break;
                case 10:
                    returnTargets.leftTarget = new Pose2d(11.699, 4.2681, Rotation2d.fromDegrees(0.7));
                    returnTargets.rightTarget = new Pose2d(11.748, 3.888, Rotation2d.fromDegrees(1.099));
                    returnTargets.centerTarget = new Pose2d(11.728, 4.055, Rotation2d.fromDegrees(0.099));
                    break;
                case 11:
                    returnTargets.leftTarget = new Pose2d(12.1567, 2.883, Rotation2d.fromDegrees(59.15));
                    returnTargets.rightTarget = new Pose2d(12.558, 2.726, Rotation2d.fromDegrees(59.58));
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
                    returnTargets.leftTarget = new Pose2d(3.5022, 2.954, Rotation2d.fromDegrees(59.690));
                    returnTargets.rightTarget = new Pose2d(3.887, 2.798, Rotation2d.fromDegrees(61.74));
                    returnTargets.centerTarget = new Pose2d(3.810, 2.899, Rotation2d.fromDegrees(60.715));
                    break;
                case 18:
                    returnTargets.leftTarget = new Pose2d(3.092, 4.3189, Rotation2d.fromDegrees(-3.02));
                    returnTargets.rightTarget = new Pose2d(3.25798, 3.9374, Rotation2d.fromDegrees(-2.9));
                    returnTargets.centerTarget = new Pose2d(3.175, 4.112, Rotation2d.fromDegrees(-2.96));
                    break;
                case 19:
                    returnTargets.leftTarget = new Pose2d(4.1496, 5.2484, Rotation2d.fromDegrees(-60.999));
                    returnTargets.rightTarget = new Pose2d(3.889, 5.0256, Rotation2d.fromDegrees(-61.54));
                    returnTargets.centerTarget = new Pose2d(3.868, 5.180, Rotation2d.fromDegrees(-61.2695));
                    break;
                case 20:
                    returnTargets.leftTarget = new Pose2d(5.3588, 4.7963, Rotation2d.fromDegrees(-122.76));
                    returnTargets.rightTarget = new Pose2d(5.02247, 4.94, Rotation2d.fromDegrees(-121.37));
                    returnTargets.centerTarget = new Pose2d(5.225, 5.122, Rotation2d.fromDegrees(-122.065));
                    break;
                case 21:
                    returnTargets.leftTarget = new Pose2d(5.799, 3.6878, Rotation2d.fromDegrees(-176.6));
                    returnTargets.rightTarget = new Pose2d(5.801, 4.081, Rotation2d.fromDegrees(-179.78));
                    returnTargets.centerTarget = new Pose2d(5.8, 3.958, Rotation2d.fromDegrees(-178.19));
                    break;
                case 22:
                    returnTargets.leftTarget = new Pose2d(4.5393, 2.91, Rotation2d.fromDegrees(115.406));
                    returnTargets.rightTarget = new Pose2d(4.6889, 3.212, Rotation2d.fromDegrees(118.77));
                    returnTargets.centerTarget = new Pose2d(4.6141, 2.80, Rotation2d.fromDegrees(117.088));
                    break;
                default:
                    returnTargets.leftTarget = null;
                    returnTargets.rightTarget = null;
                    returnTargets.centerTarget = null;
                    break;
            }
        }   

        //Adjustment code for trimming fields. 
       /* returnTargets.leftTarget = addXYToPose(returnTargets.leftTarget, trimX, trimY, trimTheta);
        returnTargets.rightTarget = addXYToPose(returnTargets.rightTarget, trimX, trimY, trimTheta);
        returnTargets.centerTarget = addXYToPose(returnTargets.centerTarget, trimX, trimY, trimTheta);
 */
        return returnTargets;
    }

    public Vision(int index) {
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
