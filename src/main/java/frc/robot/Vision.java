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

//import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

//import java.util.ArrayList;
//import org.photonvision.targeting.PhotonPipelineResult;
import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Vision {
    public final PhotonCamera camera;
    public final PhotonPoseEstimator photonEstimator;
    //public final PhotonPoseEstimator photonEstimatorSingleTarget;
    private Matrix<N3, N1> curStdDevs;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    //Vision offset calculations
    List<Pose2d> leftBranchCoordinates = null;
    List<Pose2d> rightBranchCoordinates = null; 
    List<Pose2d> offReefCoordinates = null;

    // Lookup table for angles
    private Map<Integer, Double> angleLookupTable;
    List<Integer> reefTagIds = List.of(6, 7, 8, 9, 10, 11,17, 18, 19, 20, 21, 22);

    private void populateAngleLookupTable() {

        angleLookupTable = new HashMap<Integer, Double>();
        // Populate the lookup table with values for 17-22 and 6-11
        angleLookupTable.put(17, 60.0);
        angleLookupTable.put(18, 0.0);
        angleLookupTable.put(19, -60.0);
        angleLookupTable.put(20, -120.0);
        angleLookupTable.put(21, 180.0);
        angleLookupTable.put(22, 120.0);
        angleLookupTable.put(6, 120.0);
        angleLookupTable.put(7, 180.0);
        angleLookupTable.put(8, -120.0);
        angleLookupTable.put(9, -60.0);
        angleLookupTable.put(10, 0.0);
        angleLookupTable.put(11, 60.0);
    }

    /*public static List<Pose2d> loadAprilTagPoses(List<Integer> tagIds) {
        List<Pose2d> tagPoses2d = new ArrayList<>();

        try {
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();

            for (int tagId : tagIds) {
                Optional<Pose3d> pose3dOpt = fieldLayout.getTagPose(tagId);

                if (pose3dOpt.isPresent()) {
                    Pose3d pose3d = pose3dOpt.get();

                    // Convert 3D pose to 2D (drops Z and roll/pitch)
                    Pose2d pose2d = new Pose2d(
                        pose3d.getX(),
                        pose3d.getY(),
                        new Rotation2d(pose3d.getRotation().getZ())
                    );

                    tagPoses2d.add(pose2d);
                } else {
                    System.out.println("AprilTag " + tagId + " pose not found!");
                }
            }
        } catch (Exception e) {
            System.out.println("Error loading AprilTag field layout: " + e.getMessage());
        }

        return tagPoses2d;
    }*/

    public double normalizeAngle(double angle) {
        // Normalize the angle to be within the range [0, 360)
        angle = angle % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public double AngleDifference(double robotAngle, int tag) {
        double normalizeAngle = normalizeAngle(robotAngle);
        double normalizeTagAngle = normalizeAngle(getAngleForIndex(tag));

        return Math.abs(normalizeAngle - normalizeTagAngle);

    }

    public int CompareAngles(double robotAngle, double angle1, double angle2) {
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

    public double PercentageAngle(double robotAngle, int tag) {
        double normalizeAngle = normalizeAngle(robotAngle);
        double normalizeTagAngle = normalizeAngle(getAngleForIndex(tag));

        // Don't divide by 0
        if (normalizeAngle == 0)
            normalizeAngle = 0.1;
        if (normalizeTagAngle == 0.0)
            normalizeTagAngle = 0.1;

        return normalizeAngle / normalizeTagAngle;
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

    public Pose2d GetDestinationFromReefBranch(int selectedReefBranch, Constants.Alignment alignment) {

        if (alignment == Constants.Alignment.BRANCH) 
            return GetDestinationFromReefStandard(selectedReefBranch);
        else if (alignment == Constants.Alignment.LEVEL1)
            return GetDestinationFromReefLevel1(selectedReefBranch);
        else if (alignment == Constants.Alignment.ALGEA)
            return GetDestinationFromReefCenter(selectedReefBranch);
        else if (alignment == Constants.Alignment.OFF_REEF)
            return GetOffReefDestination(selectedReefBranch);
        else
            return null;
    }


    /**
     * Returns the AprilTag ID associated with a reef branch ID (1–12).
     * Uses DriverStation alliance to determine Red or Blue reef.
     *
     * @param branchID A number from 1 to 12 representing reef segment
     * @return AprilTag ID corresponding to the given branch and alliance
     */
    public static int getReefTagId(int branchID) {
        if (branchID < 1 || branchID > 12) {
            throw new IllegalArgumentException("branchID must be between 1 and 12.");
        }

        // Tag IDs by face: [12:00, 10:00, 8:00, 6:00, 4:00, 2:00]
        int[] blueTags = {21, 20, 19, 18, 17, 22};
        int[] redTags  = {10, 11, 6, 7, 8, 9};

        // Get alliance (default to Red if not connected)
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

        // Each face has 2 branches → faceIndex from 0 to 5
        int faceIndex = (branchID - 1) / 2;

        return (alliance == Alliance.Blue) ? blueTags[faceIndex] : redTags[faceIndex];
    }

    public int GetAprilTagFromReefBranchId(int selectedReefBranch)
    {
        int aprilTag = 0;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.get() == Alliance.Red) {
            switch (selectedReefBranch) {
                case Constants.ReefOperatorConstants.TWELVE_LEFT:
                case Constants.ReefOperatorConstants.TWELVE_RIGHT:
                    aprilTag = 10;
                    break;
                case Constants.ReefOperatorConstants.TEN_LEFT:
                case Constants.ReefOperatorConstants.TEN_RIGHT:
                    aprilTag = 11;
                    break;
                case Constants.ReefOperatorConstants.EIGHT_LEFT:
                case Constants.ReefOperatorConstants.EIGHT_RIGHT:
                    aprilTag = 6;
                    break;
                case Constants.ReefOperatorConstants.SIX_LEFT:
                case Constants.ReefOperatorConstants.SIX_RIGHT:
                    aprilTag = 7;
                    break;
                case Constants.ReefOperatorConstants.FOUR_LEFT:
                case Constants.ReefOperatorConstants.FOUR_RIGHT:
                    aprilTag = 8;
                    break;
                case Constants.ReefOperatorConstants.TWO_LEFT:
                case Constants.ReefOperatorConstants.TWO_RIGHT:
                    aprilTag = 9;
                    break;
            }
        } else {
            switch (selectedReefBranch) {
                case Constants.ReefOperatorConstants.TWELVE_LEFT:
                case Constants.ReefOperatorConstants.TWELVE_RIGHT:
                    aprilTag = 21;
                    break;
                case Constants.ReefOperatorConstants.TEN_LEFT:
                case Constants.ReefOperatorConstants.TEN_RIGHT:
                    aprilTag = 20;
                    break;
                case Constants.ReefOperatorConstants.EIGHT_LEFT:
                case Constants.ReefOperatorConstants.EIGHT_RIGHT:
                    aprilTag = 19;
                    break;
                case Constants.ReefOperatorConstants.SIX_LEFT:
                case Constants.ReefOperatorConstants.SIX_RIGHT:
                    aprilTag = 18;
                    break;
                case Constants.ReefOperatorConstants.FOUR_LEFT:
                case Constants.ReefOperatorConstants.FOUR_RIGHT:
                    aprilTag = 17;
                    break;
                case Constants.ReefOperatorConstants.TWO_LEFT:
                case Constants.ReefOperatorConstants.TWO_RIGHT:
                    aprilTag = 22;
                    break;

            }
        }
        return aprilTag;
    }
    public Pose2d GetOffReefDestination(int selectedReefBranch)
    {
        Pose2d destination = null;
        int aprilTag = GetAprilTagFromReefBranchId(selectedReefBranch);
        Optional<Alliance> ally = DriverStation.getAlliance();

        if (ally.get() == Alliance.Red)
        {
            switch(aprilTag)
            {
                case 10:
                    destination = new Pose2d(11.57, 4.10, Rotation2d.fromDegrees(0));
                    break;
                case 11:
                    destination = new Pose2d(12.24, 2.77, Rotation2d.fromDegrees(60));
                    break;
                case 6:
                    destination = new Pose2d(13.73, 2.69, Rotation2d.fromDegrees(120));
                    break;
                case 7:
                    destination = new Pose2d(14.55, 3.94, Rotation2d.fromDegrees(180));
                    break;
                case 8:
                    destination = new Pose2d(13.87, 5.27, Rotation2d.fromDegrees(-120));
                    break;
                case 9:
                    destination = new Pose2d(12.39, 5.35, Rotation2d.fromDegrees(-60));
                    break;
            }
        }
        else //blue
        {
            switch(aprilTag)
            {
                case 21:
                    destination = new Pose2d(5.98, 3.94, Rotation2d.fromDegrees(180));
                    break;
                case 20:
                    destination = new Pose2d(5.305, 5.507, Rotation2d.fromDegrees(-120));
                    break;
                case 19:
                    destination = new Pose2d(3.82, 5.35, Rotation2d.fromDegrees(-60));
                    break;
                case 18:
                    destination = new Pose2d(3, 4.103, Rotation2d.fromDegrees(0));
                    break;
                case 17:
                    destination = new Pose2d(3.67, 2.77, Rotation2d.fromDegrees(60));
                    break;
                case 22:
                    destination = new Pose2d(5.16, 2.69, Rotation2d.fromDegrees(120));
                    break;
            }
        }
        
        return destination;
    }
    public Pose2d GetDestinationFromReefCenter(int selectedReefBranch) {
        Pose2d destination = null;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.get() == Alliance.Red) {
            switch (selectedReefBranch) {
                case Constants.ReefOperatorConstants.TWELVE_LEFT: // 10 left
                case Constants.ReefOperatorConstants.TWELVE_RIGHT: // 10 right
                    //destination = new Pose2d(11.728, 4.055, Rotation2d.fromDegrees(0.099));
                    destination = new Pose2d(11.596, 4.004, Rotation2d.fromDegrees(0));
                    break;
                case Constants.ReefOperatorConstants.TEN_LEFT: // 11 left
                case Constants.ReefOperatorConstants.TEN_RIGHT: // 11 right
                    //destination = new Pose2d(12.422, 2.88, Rotation2d.fromDegrees(59.365));  //original
                    destination = new Pose2d(12.328, 2.763, Rotation2d.fromDegrees(60));
                    break;
                case Constants.ReefOperatorConstants.EIGHT_LEFT: // 6 left
                case Constants.ReefOperatorConstants.EIGHT_RIGHT: // 6 right
                    destination = new Pose2d(13.759, 2.763, Rotation2d.fromDegrees(120));
                    break;
                case Constants.ReefOperatorConstants.SIX_LEFT: // 7 left
                case Constants.ReefOperatorConstants.SIX_RIGHT: // 7 right
                    //destination = new Pose2d(14.437, 3.996, Rotation2d.fromDegrees(177.417));  //original
                    destination = new Pose2d(14.437, 3.996, Rotation2d.fromDegrees(180));
                    break;
                case Constants.ReefOperatorConstants.FOUR_LEFT: // 8 left
                case Constants.ReefOperatorConstants.FOUR_RIGHT: // 8 right
                    //destination = new Pose2d(14.437, 3.996, Rotation2d.fromDegrees(177.417));  //original
                    destination = new Pose2d(13.824, 5.311, Rotation2d.fromDegrees(-120));
                    break;
                case Constants.ReefOperatorConstants.TWO_LEFT: // 9 legy
                case Constants.ReefOperatorConstants.TWO_RIGHT: // 9 right
                    //destination = new Pose2d(14.437, 3.996, Rotation2d.fromDegrees(177.417)); //original
                    destination = new Pose2d(12.328, 5.325, Rotation2d.fromDegrees(-60));
                    break;
            }
        } else {
            switch (selectedReefBranch) {
                case Constants.ReefOperatorConstants.TWELVE_LEFT: // 21 left
                case Constants.ReefOperatorConstants.TWELVE_RIGHT: // 21 right
                    //destination = new Pose2d(5.8, 3.958, Rotation2d.fromDegrees(-178.19));
                    destination = new Pose2d(5.957, 4.012, Rotation2d.fromDegrees(180));
                    break;
                case Constants.ReefOperatorConstants.TEN_LEFT: // 20 left
                case Constants.ReefOperatorConstants.TEN_RIGHT: // 20 right
                    //destination = new Pose2d(5.225, 5.122, Rotation2d.fromDegrees(-122.065));
                    destination = new Pose2d(5.284, 5.260, Rotation2d.fromDegrees(-120));
                    break;
                case Constants.ReefOperatorConstants.EIGHT_LEFT: // 19 left
                case Constants.ReefOperatorConstants.EIGHT_RIGHT: // 19 right
                    //destination = new Pose2d(3.737354424938496, 4.940004156734, Rotation2d.fromDegrees(-61.2695));
                    //destination = new Pose2d(3.742, 5.319, Rotation2d.fromDegrees(-60));
                    destination = new Pose2d(3.767, 5.319, Rotation2d.fromDegrees(-60));
                    break;
                case Constants.ReefOperatorConstants.SIX_LEFT: // 18 left
                case Constants.ReefOperatorConstants.SIX_RIGHT: // 18 right
                    //destination = new Pose2d(3.175, 4.112, Rotation2d.fromDegrees(-2.96));
                    destination = new Pose2d(3.016, 4.019, Rotation2d.fromDegrees(0));
                    break;
                case Constants.ReefOperatorConstants.FOUR_LEFT: // 17 left
                case Constants.ReefOperatorConstants.FOUR_RIGHT: // 17 right
                    //destination = new Pose2d(3.810, 2.899, Rotation2d.fromDegrees(60.715));
                    destination = new Pose2d(3.728, 2.751, Rotation2d.fromDegrees(60));
                    break;
                case Constants.ReefOperatorConstants.TWO_LEFT: // 22 left
                case Constants.ReefOperatorConstants.TWO_RIGHT: // 22 right
                    //destination = new Pose2d(4.6141, 2.80, Rotation2d.fromDegrees(117.088));
                    destination = new Pose2d(5.225, 2.75, Rotation2d.fromDegrees(120));
                    break;
                default:
                    destination = null;
                    break;
            }
        }

        return destination;
    }

    public Pose2d GetDestinationFromReefLevel1(int selectedReefBranch) {
        Pose2d destination = null;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.get() == Alliance.Red) {
            switch (selectedReefBranch) {
                case Constants.ReefOperatorConstants.TWELVE_LEFT: // 10 left
                case Constants.ReefOperatorConstants.TWELVE_RIGHT: // 10 right
                    //destination = new Pose2d(11.728, 4.055, Rotation2d.fromDegrees(0.099));
                    destination = new Pose2d(11.596, 4.004, Rotation2d.fromDegrees(0));
                    break;
                case Constants.ReefOperatorConstants.TEN_LEFT: // 11 left
                case Constants.ReefOperatorConstants.TEN_RIGHT: // 11 right
                    //destination = new Pose2d(12.422, 2.88, Rotation2d.fromDegrees(59.365));  //original
                    destination = new Pose2d(12.328, 2.763, Rotation2d.fromDegrees(60));
                    break;
                case Constants.ReefOperatorConstants.EIGHT_LEFT: // 6 left
                case Constants.ReefOperatorConstants.EIGHT_RIGHT: // 6 right
                    destination = new Pose2d(13.759, 2.763, Rotation2d.fromDegrees(120));
                    break;
                case Constants.ReefOperatorConstants.SIX_LEFT: // 7 left
                case Constants.ReefOperatorConstants.SIX_RIGHT: // 7 right
                    //destination = new Pose2d(14.437, 3.996, Rotation2d.fromDegrees(177.417));  //original
                    destination = new Pose2d(14.437, 3.996, Rotation2d.fromDegrees(180));
                    break;
                case Constants.ReefOperatorConstants.FOUR_LEFT: // 8 left
                case Constants.ReefOperatorConstants.FOUR_RIGHT: // 8 right
                    //destination = new Pose2d(14.437, 3.996, Rotation2d.fromDegrees(177.417));  //original
                    destination = new Pose2d(13.824, 5.311, Rotation2d.fromDegrees(-120));
                    break;
                case Constants.ReefOperatorConstants.TWO_LEFT: // 9 legy
                case Constants.ReefOperatorConstants.TWO_RIGHT: // 9 right
                    //destination = new Pose2d(14.437, 3.996, Rotation2d.fromDegrees(177.417)); //original
                    destination = new Pose2d(12.328, 5.325, Rotation2d.fromDegrees(-60));
                    break;
                default:
                    destination = null;
                    break;
            }
        } else {
            switch (selectedReefBranch) {
                case Constants.ReefOperatorConstants.TWELVE_LEFT: // 21 left
                case Constants.ReefOperatorConstants.TWELVE_RIGHT: // 21 right
                    //destination = new Pose2d(5.8, 3.958, Rotation2d.fromDegrees(-178.19));
                    destination = new Pose2d(5.957, 4.012, Rotation2d.fromDegrees(180));
                    break;
                case Constants.ReefOperatorConstants.TEN_LEFT: // 20 left
                case Constants.ReefOperatorConstants.TEN_RIGHT: // 20 right
                    //destination = new Pose2d(5.225, 5.122, Rotation2d.fromDegrees(-122.065));
                    destination = new Pose2d(5.284, 5.260, Rotation2d.fromDegrees(-120));
                    break;
                case Constants.ReefOperatorConstants.EIGHT_LEFT: // 19 left
                case Constants.ReefOperatorConstants.EIGHT_RIGHT: // 19 right
                    //destination = new Pose2d(3.737354424938496, 4.940004156734, Rotation2d.fromDegrees(-61.2695));
                    destination = new Pose2d(3.742, 5.319, Rotation2d.fromDegrees(-60));
                    break;
                case Constants.ReefOperatorConstants.SIX_LEFT: // 18 left
                case Constants.ReefOperatorConstants.SIX_RIGHT: // 18 right
                    //destination = new Pose2d(3.175, 4.112, Rotation2d.fromDegrees(-2.96));
                    destination = new Pose2d(3.016, 4.019, Rotation2d.fromDegrees(0));
                    break;
                case Constants.ReefOperatorConstants.FOUR_LEFT: // 17 left
                case Constants.ReefOperatorConstants.FOUR_RIGHT: // 17 right
                    //destination = new Pose2d(3.810, 2.899, Rotation2d.fromDegrees(60.715));
                    destination = new Pose2d(3.728, 2.751, Rotation2d.fromDegrees(60));
                    break;
                case Constants.ReefOperatorConstants.TWO_LEFT: // 22 left
                case Constants.ReefOperatorConstants.TWO_RIGHT: // 22 right
                    //destination = new Pose2d(4.6141, 2.80, Rotation2d.fromDegrees(117.088));
                    destination = new Pose2d(5.225, 2.75, Rotation2d.fromDegrees(120));
                    break;
                default:
                    destination = null;
                    break;
            }
        }

        return destination;
    }

    public Pose2d GetOffReefDestinationEx(int selectedReefBranch)
    {
        Pose2d destination = null;
        Optional<Alliance> ally = DriverStation.getAlliance();
        int aprilTag = GetAprilTagFromReefBranchId(selectedReefBranch);
        int offsetIndex = reefTagIds.indexOf(aprilTag);

        if (ally.get() == Alliance.Red)
        {
            switch(aprilTag)
            {
                
            }
        }
        else //blue
        {
            switch(aprilTag)
            {
                case 18:
                    Pose2d tempdestination = offReefCoordinates.get(offsetIndex);
                    destination = new Pose2d(tempdestination.getX(), tempdestination.getY(), Rotation2d.fromDegrees(0));
                    break;
            }
        }

        return destination;
    }

    public Pose2d GetDestinationFromReefStandardEx(int selectedReefBranch) {
        Pose2d destination = null;
        Optional<Alliance> ally = DriverStation.getAlliance();

        int tagIndex = GetAprilTagFromReefBranchId(selectedReefBranch);
        int offsetIndex = reefTagIds.indexOf(tagIndex);

        if (ally.get() == Alliance.Red) 
        {
            switch (selectedReefBranch) {}
            
        }
        else
        {
            switch (selectedReefBranch) {
                case Constants.ReefOperatorConstants.SIX_LEFT: // 7 left
                
                    destination = leftBranchCoordinates.get(offsetIndex);
                    break;
                case Constants.ReefOperatorConstants.SIX_RIGHT: // 7 right
                    destination = rightBranchCoordinates.get(offsetIndex);
                    break;
            }
        }

        return destination;

    }
    public Pose2d GetDestinationFromReefStandard(int selectedReefBranch) {
        Pose2d destination = null;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.get() == Alliance.Red) {
            switch (selectedReefBranch) {
                case Constants.ReefOperatorConstants.TWELVE_LEFT: // 10 left Updated 8/3
                    destination = new Pose2d(11.568, 4.31, Rotation2d.fromDegrees(0));
                    break;
                case Constants.ReefOperatorConstants.TWELVE_RIGHT: // 10 right  Updated 8/3
                    destination = new Pose2d(11.568, 3.969, Rotation2d.fromDegrees(0));
                    break;
                case Constants.ReefOperatorConstants.TEN_LEFT: // 11 left Updated 8/3
                    destination = new Pose2d(12.04, 2.831, Rotation2d.fromDegrees(60));
                    break;
                case Constants.ReefOperatorConstants.TEN_RIGHT: // 11 right 
                    destination = new Pose2d(12.293, 2.649, Rotation2d.fromDegrees(60));
                    break;
                case Constants.ReefOperatorConstants.EIGHT_LEFT: // 6 left  Updated 8/3
                    destination = new Pose2d(13.583, 2.595, Rotation2d.fromDegrees(120));
                    break;
                case Constants.ReefOperatorConstants.EIGHT_RIGHT: // 6 right  Updated 8/3
                    destination = new Pose2d(13.945, 2.770, Rotation2d.fromDegrees(120));
                    break;
                case Constants.ReefOperatorConstants.SIX_LEFT: // 7 left Updated 8/3
                    destination = new Pose2d(14.566, 3.7575, Rotation2d.fromDegrees(180));
                    break;
                case Constants.ReefOperatorConstants.SIX_RIGHT: // 7 right Updated 8/3
                    destination = new Pose2d(14.566, 4.0985, Rotation2d.fromDegrees(180));
                    break;
                case Constants.ReefOperatorConstants.FOUR_LEFT: // 8 left Updated 8/3
                    destination = new Pose2d(14.0715, 5.225, Rotation2d.fromDegrees(-120));
                    break;
                case Constants.ReefOperatorConstants.FOUR_RIGHT: // 8 right Updated 8/3
                    destination = new Pose2d(13.791, 5.363, Rotation2d.fromDegrees(-120));
                    break;
                case Constants.ReefOperatorConstants.TWO_LEFT: // 9 left Updated 8/3
                    destination = new Pose2d(12.570, 5.469, Rotation2d.fromDegrees(-60));
                    break;
                case Constants.ReefOperatorConstants.TWO_RIGHT: // 9 right Updated 8/3
                    destination = new Pose2d(12.17375, 5.332, Rotation2d.fromDegrees(-60));
                    break;
                default:
                    destination = null;
                    break;
            }
        } else {
            switch (selectedReefBranch) {
                case Constants.ReefOperatorConstants.TWELVE_LEFT: // 21 left UPDATED 8/3
                    destination = new Pose2d(5.984, 3.775, Rotation2d.fromDegrees(180));
                    break;
                case Constants.ReefOperatorConstants.TWELVE_RIGHT: // 21 right UPDATED 8/3
                    destination = new Pose2d(5.88, 4.07, Rotation2d.fromDegrees(-180));
                    break;
                case Constants.ReefOperatorConstants.TEN_LEFT: // 20 left  UPDATED 8/3
                    destination = new Pose2d(5.486, 5.225, Rotation2d.fromDegrees(-120));
                    break;
                case Constants.ReefOperatorConstants.TEN_RIGHT: // 20 right
                    destination = new Pose2d(5.240, 5.416, Rotation2d.fromDegrees(-120));
                    break;
                case Constants.ReefOperatorConstants.EIGHT_LEFT: // 19 left UPDATED 8/3
                    destination = new Pose2d(3.93,5.416,Rotation2d.fromDegrees((-60)));
                    break;
                case Constants.ReefOperatorConstants.EIGHT_RIGHT: // 19 right UPDATED 8/3
                    destination = new Pose2d(3.702, 5.197, Rotation2d.fromDegrees(-60));
                    break;
                case Constants.ReefOperatorConstants.SIX_LEFT: // 18 left UPDATED 8/3
                    destination = new Pose2d(3.0, 4.29, Rotation2d.fromDegrees(0));
                    break;
                case Constants.ReefOperatorConstants.SIX_RIGHT: // 18 right UPDATED 8/3
                    destination = new Pose2d(3.1, 3.93, Rotation2d.fromDegrees(0));
                    break;
                case Constants.ReefOperatorConstants.FOUR_LEFT: // 17 left UPDATED 8/3
                    destination = new Pose2d(3.51, 2.897, Rotation2d.fromDegrees(60));
                    break;
                case Constants.ReefOperatorConstants.FOUR_RIGHT: // 17 right UPDATED 8/3
                    destination = new Pose2d(3.84, 2.79, Rotation2d.fromDegrees(60));
                    break;
                case Constants.ReefOperatorConstants.TWO_LEFT: // 22 left UPDATED 8/3
                    destination = new Pose2d(5.000, 2.633, Rotation2d.fromDegrees(120)); 
                    break;
                case Constants.ReefOperatorConstants.TWO_RIGHT: // 22 right UPDATED 8/3
                    destination = new Pose2d(5.33, 2.8, Rotation2d.fromDegrees(120));
                    break;
                default:
                    destination = null;
                    break;
            }
        }

        return destination;
    }

    public static void printTagPoses(List<Integer> tagIds, List<Pose2d> poses) {
        if (tagIds.size() != poses.size()) {
            System.out.println("Error: tag ID list and pose list are not the same size!");
            return;
        }

        for (int i = 0; i < poses.size(); i++) {
            int tagId = tagIds.get(i);
            Pose2d pose = poses.get(i);

            double x = pose.getX();
            double y = pose.getY();
            double headingDeg = pose.getRotation().getDegrees();

            System.out.printf(
                "AprilTag %d: X = %.2f m, Y = %.2f m, Rotation = %.1f°%n",
                tagId, x, y, headingDeg
            );
        }
    }

    public static void printTag18Poses(List<Integer> tagIds, List<Pose2d> poses) {
        if (tagIds.size() != poses.size()) {
            System.out.println("Error: tag ID list and pose list are not the same size!");
            return;
        }

        for (int i = 0; i < poses.size(); i++) {
            int tagId = tagIds.get(i);
            Pose2d pose = poses.get(i);

            double x = pose.getX();
            double y = pose.getY();
            double headingDeg = pose.getRotation().getDegrees();

            System.out.printf(
                "AprilTag %d: X = %.2f m, Y = %.2f m, Rotation = %.1f°%n",
                tagId, x, y, headingDeg
            );
        }
    }


    /**
     * Calculates offset poses relative to a list of AprilTag IDs.
     * Offsets are applied in the tag's local 2D coordinate frame (horizontal = forward, vertical = left).
     *
     * @param tagIds           List of AprilTag IDs
     * @param horizontalOffset Offset in meters forward/backward (along tag's X)
     * @param verticalOffset   Offset in meters left/right (along tag's Y)
     * @return List of Pose2d with the given offsets applied
     */
    /*public static List<Pose2d> calculateOffsetsWithXY(List<Integer> tagIds, double horizontalOffset, double verticalOffset) {
        List<Pose2d> offsetPoses = new ArrayList<>();

        try {
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();

            for (int tagId : tagIds) {
                Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);

                if (tagPoseOpt.isPresent()) {
                    Pose3d tagPose3d = tagPoseOpt.get();

                    // Convert Pose3d to Pose2d using only X, Y, and yaw (Z-axis rotation)
                    double x = tagPose3d.getX();
                    double y = tagPose3d.getY();
                    Rotation3d rot3d = tagPose3d.getRotation();
                    Rotation2d heading = new Rotation2d(rot3d.getZ());  // assumes Z is yaw in radians

                    // Apply offset relative to tag's heading
                    double offsetX = horizontalOffset * heading.getCos() - verticalOffset * heading.getSin();
                    double offsetY = horizontalOffset * heading.getSin() + verticalOffset * heading.getCos();

                    Pose2d offsetPose = new Pose2d(
                        x + offsetX,
                        y + offsetY,
                        heading  // Keep same rotation
                    );

                    offsetPoses.add(offsetPose);
                } else {
                    System.out.println("AprilTag " + tagId + " not found in field layout!");
                }
            }
        } catch (Exception e) {
            System.out.println("Error loading AprilTag field layout: " + e.getMessage());
        }

        return offsetPoses;
    }*/

    public Vision(int index) {

        //Grab the base tag IDs
        
        //List<Pose2d> poses = loadAprilTagPoses(reefTagIds);
        //printTagPoses(reefTagIds, poses);

        //Generate the list of coordinates based onoffsets. 
        //todo: move to constants
        /*double leftBranchHorz = 0.56;
        double leftBranchVert = -0.3;
        double rightBranchHorz = 0.56;
        double rightBranchVert = 0.05;
        double offReedHorz = 0.658;
        double offReefVert = -0.082;
        
        leftBranchCoordinates = calculateOffsetsWithXY(reefTagIds, leftBranchHorz, leftBranchVert);
        rightBranchCoordinates = calculateOffsetsWithXY(reefTagIds, rightBranchHorz, rightBranchVert);
        offReefCoordinates = calculateOffsetsWithXY(reefTagIds, offReedHorz, offReefVert);

        printTagPoses(reefTagIds, offReefCoordinates);*/

        /*int tagIndex = reefTagIds.indexOf(19);
        Pose2d tag18OffsetPose = leftBranchCoordinates.get(tagIndex);
        System.out.printf("Left Offset pose for Tag 18: (X=%.2f, Y=%.2f, Rot=%.1f°)%n",
            tag18OffsetPose.getX(),
            tag18OffsetPose.getY(),
            tag18OffsetPose.getRotation().getDegrees());

        tag18OffsetPose = rightBranchCoordinates.get(tagIndex);
            System.out.printf("Right Offset pose for Tag 18: (X=%.2f, Y=%.2f, Rot=%.1f°)%n",
                tag18OffsetPose.getX(),
                tag18OffsetPose.getY(),
                tag18OffsetPose.getRotation().getDegrees());

        tag18OffsetPose = offReefCoordinates.get(tagIndex);
        System.out.printf("Offreef pose for Tag 18: (X=%.2f, Y=%.2f, Rot=%.1f°)%n",
            tag18OffsetPose.getX(),
            tag18OffsetPose.getY(),
            tag18OffsetPose.getRotation().getDegrees());*/
    
        populateAngleLookupTable();
        camera = new PhotonCamera(Constants.Vision.CamNames[index]);

        //photonEstimatorSingleTarget = new PhotonPoseEstimator(kTagLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
        //        cameraTransforms[index]);
        //photonEstimatorSingleTarget.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR   /* PNP_DISTANCE_TRIG_SOLVE MULTI_TAG_PNP_ON_COPROCESSOR*/,
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
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(boolean singleTarget) {

        PhotonPoseEstimator photonEstimatorToUse;
       // if(singleTarget)
        //    photonEstimatorToUse = photonEstimatorSingleTarget;
        //else
            photonEstimatorToUse = photonEstimator;
        
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimatorToUse.update(change);
            updateEstimationStdDevs(singleTarget,visionEst, change.getTargets());

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
    private void updateEstimationStdDevs(boolean singleTarget,
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

        PhotonPoseEstimator photonEstimatorToUse;
        //if(singleTarget)
        //    photonEstimatorToUse = photonEstimatorSingleTarget;
       // else
            photonEstimatorToUse = photonEstimator;

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
                var tagPose = photonEstimatorToUse.getFieldTags().getTagPose(tgt.getFiducialId());
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