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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import java.util.Optional;
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

    public ReefTargets CalculateAutoReefTarget(int aprilTag) {
        // aprilTag = Robot.aprilTagId;
        Optional<Alliance> ally = DriverStation.getAlliance();
        ReefTargets returnTargets = new ReefTargets();
        if (ally.get() == Alliance.Red) {
            switch (aprilTag) {
                case 6:
                    returnTargets.leftTarget = new Pose2d(11.356, 6.121, Rotation2d.fromDegrees(-54.893));
                    returnTargets.rightTarget = new Pose2d(13.825, 2.631, Rotation2d.fromDegrees(-49.185));
                    returnTargets.centerTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                    break;
                case 7:
                    returnTargets.leftTarget = new Pose2d(15.5, 3.6925, Rotation2d.fromDegrees(174.06));
                    returnTargets.rightTarget = new Pose2d(15.5, 4.355, Rotation2d.fromDegrees(174.06));
                    returnTargets.centerTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                    break;
                case 8:
                    returnTargets.leftTarget = new Pose2d(11.3, 0.408, Rotation2d.fromDegrees(-110.383));
                    returnTargets.rightTarget = new Pose2d(13.523, 5.566, Rotation2d.fromDegrees(-110.383));
                    returnTargets.centerTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                    break;
                case 9:
                    returnTargets.leftTarget = new Pose2d(12.314, 5.371, Rotation2d.fromDegrees(-58.392));
                    returnTargets.rightTarget = new Pose2d(12.207, 5.322, Rotation2d.fromDegrees(-58.392));
                    returnTargets.centerTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                    break;
                case 10:
                    returnTargets.leftTarget = new Pose2d(11.459, 3.890, Rotation2d.fromDegrees(-25.0));
                    returnTargets.rightTarget = new Pose2d(14.515, 4.002, Rotation2d.fromDegrees(-25.0));
                    returnTargets.centerTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                    break;
                case 11:
                    returnTargets.leftTarget = new Pose2d(12.268, 2.688, Rotation2d.fromDegrees(-25.0));
                    returnTargets.rightTarget = new Pose2d(14.515, 4.002, Rotation2d.fromDegrees(-25.0));
                    returnTargets.centerTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
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
                    returnTargets.leftTarget = new Pose2d(3.695, 2.726, Rotation2d.fromDegrees(-25.0));
                    returnTargets.rightTarget = new Pose2d(13.932, 2.736, Rotation2d.fromDegrees(85.601));
                    returnTargets.centerTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                    break;
                case 18:
                    returnTargets.leftTarget = new Pose2d(2.906, 3.938, Rotation2d.fromDegrees(-25.0));
                    returnTargets.rightTarget = new Pose2d(14.515, 4.044, Rotation2d.fromDegrees(-165.174));
                    returnTargets.centerTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                    break;
                case 19:
                    returnTargets.leftTarget = new Pose2d(3.685, 5.362, Rotation2d.fromDegrees(-25.0));
                    returnTargets.rightTarget = new Pose2d(13.759, 5.449, Rotation2d.fromDegrees(-110.61));
                    returnTargets.centerTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                    break;
                case 20:
                    returnTargets.leftTarget = new Pose2d(5.263, 5.314, Rotation2d.fromDegrees(-25.0));
                    returnTargets.rightTarget = new Pose2d(14.515, 4.002, Rotation2d.fromDegrees(-25.0));
                    returnTargets.centerTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                    break;
                case 21:
                    returnTargets.leftTarget = new Pose2d(5.985, 4.063, Rotation2d.fromDegrees(-25.0));
                    returnTargets.rightTarget = new Pose2d(14.515, 4.002, Rotation2d.fromDegrees(-25.0));
                    returnTargets.centerTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                    break;
                case 22:
                    returnTargets.leftTarget = new Pose2d(5.186, 2.716, Rotation2d.fromDegrees(-25.0));
                    returnTargets.rightTarget = new Pose2d(14.515, 4.002, Rotation2d.fromDegrees(-25.0));
                    returnTargets.centerTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
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
        camera = new PhotonCamera(Constants.Vision.CamNames[index]);

        photonEstimator =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraTransforms[index]);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(kTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, cameraTransforms[index]);

            cameraSim.enableDrawWireframe(true);
        }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                getSimDebugField()
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
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
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

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
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
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
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
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}
