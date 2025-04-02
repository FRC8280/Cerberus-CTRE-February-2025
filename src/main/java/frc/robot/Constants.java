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

 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
 import edu.wpi.first.math.geometry.Rotation3d;
 import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.math.geometry.Translation3d;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
 
 public class Constants {

    public enum Alignment {
        BRANCH,
        LEVEL1,
        ALGEA
    }

     public static class AutoAlignment{
        public static int noReef = -1;
        public static double maxVelocity = 3;//2.25;
        public static double maxAcceleration = 3;//2.25;
        public static double MaxAngularRate = Units.degreesToRadians(200);//360;
        public static double MaxAngularAcceleration = Units.degreesToRadians(300) ;//540; 
     
        public static double AutoAlignmentSpeed = 0.60; //0.40;//0.25;
        public static double AutoReefAlignmentSpeed = 0.75;//0.50;//0.35;
    }
     public static class Vision {

        public static double CAMERA_HEIGHT_METERS = Units.inchesToMeters(11.5);
        public static double TARGET_HEIGHT_METERS = Units.inchesToMeters(51.875);
        public static double CAMERA_PITCH_RADIANS = Units.degreesToRadians(45);

        public static final String[] CamNames = {
            "FrontRightCamera", "FrontLeftCamera" //,"LeftSideCam", //"RearAngle" /*See if you can change */, 
            //"RightSideCam"//, "RearNormal" /*See if you can change*/
        };

       /* // example Cam mounted facing forward, half a meter forward of center, half a meter up from center*/
        public static final Transform3d[] cameraTransforms = {
            //FrontRightCamera
            new Transform3d(new Translation3d(Units.inchesToMeters(12.5+6), Units.inchesToMeters(-8.75), Units.inchesToMeters(8.5)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))),
            //FrontLeftCamera
            new Transform3d(new Translation3d(Units.inchesToMeters(12.5+6), Units.inchesToMeters(8.75), Units.inchesToMeters(8.5)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))),
            //LeftSideCam
          //  new Transform3d(new Translation3d(Units.inchesToMeters(3), Units.inchesToMeters(7), Units.inchesToMeters(35)),
            //    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270))),
            
            
            /* new Transform3d(new Translation3d(Units.inchesToMeters(-6), Units.inchesToMeters(3), Units.inchesToMeters(22)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180))),
            new Transform3d(new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(-12.5), Units.inchesToMeters(9.5)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90))),
            new Transform3d(new Translation3d(Units.inchesToMeters(-12.5), Units.inchesToMeters(8.5), Units.inchesToMeters(6)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-30), Units.degreesToRadians(180))),*/
        };
         // The layout of the AprilTags on the field
         public static final AprilTagFieldLayout kTagLayout =
                 AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
 
         // The standard deviations of our vision estimated poses, which affect correction rate
         // (Fake values. Experiment and determine estimation noise on an actual robot.)
         public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
         public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
     }

     public static final class Elevator {
        public static final int kFrontCanId = 11;
        public static final int kRearCanId = 12;
        public static final int kBottomLaserCanId = 38;

        public static final double levelOne = 4.5; //4.5;//3.4998; //2.03564453125; //3.4998; //idk find the values later
        public static final double levelTwo = 6.96728515625;
        public static final double levelThree = 14.49;//14.0595703125;
        public static final double levelFour = 25.45;//25.39;//25.45;
        public static final double noDestination = -999;
        public static final double kStowed = 0;

        public static final int kCurrentLimit = 60;

        public static final double kElevatorDetectionRange
         = 450;
        
    }

    public static final class Effector{
        public static final int kMotorCanId = 16;
        public static final int kAlgeaMotorCandID = 17;
        public static final double kCurrentLimit = 60;
        public static final double kVoltageComp = 12;
        public static final double kSpeed = 40;
        public static final double kSlowerSpeed = 20;
        public static final double kAdjustIntakeSpeed = -25;
        
        
        public static final int kFrontLaserCanId = 39;
        public static final int kRearLaserCanId = 40;
        public static final int kCoralDetectionRange = 120;//100;//70;
        public static final int intakeTimerMax = 3;
        public static final double intakeSpeed = 2;//50;//300; //0.5;
        public static final int ejectSpeed = -10;
        public static final int scoreSpeed = 10;

        public static final double algeaArmScorePosition = -1.34;

    }

    public static final class ClimberConstants {
        public static final int CLIMBER_MOTOR_ID = 13;
        public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 40;
        public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
        public static final double CLIMBER_SPEED_DOWN = -0.5;
        public static final double CLIMBER_SPEED_UP = 0.5;
    }

    public static final class AlgaeConstants {
         // Motors
         public static final int kRollerMotorId = 14; //todo: change to ours
         public static final int kPivotMotorId = 13; //todo: change to ours
         public static final int kLaserCanID = 41;
       
         // Absolute encoder offset
         public static final double k_pivotEncoderOffset = 0;
     
         
         // Pivot set point encoder values =
         public static final double k_pivotAngleGround = 0;
         public static final double k_pivotAngleProcessor = 0;
         public static final double k_pivotAngleStow = 0;
         public static final double k_NoteInIntakeDistance = 0;
 
     
         // Intake speeds
         public static final double k_intakeSpeed = 50;
         public static final double k_EjectSpeed = -50;
        
         /*New commit from REV */
         public static final double kArmGearRatio = 1.0 / (100); 
         public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
         public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
         public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
         public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 0.4, 12.0/kArmFreeSpeed, 0.0);
        // public static final PIDGains kArmPositionGains = new PIDGains(0.6, 0.0, 0.0);
         
         public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);
         public static final double TrapezoidProfileMaxVel = 2.0;
         public static final double TrapezoidProfileMaxAcc = 2.0;
 
         public static final double kHomePosition = 0.0;
         public static final double kIntakePosition = 0.209;
         public static final double kScoringPosition = 0.1;
 
         public static final double kSoftLimitReverse = 0; //-140;
         public static final double kSoftLimitForward = 4.77;//4.6 or 143.878
 
         public static final double kArmZeroCosineOffset = - Math.PI / 6; //radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
 
         public static final double kIntakeBarDistanceMM = 60;
     }

     public static final class ManualOperatorConstants{
        public static final int MANUAL_SWITCH = 1;
        public static final int MANUAL_SCORE = 2;
        public static final int CLIMBER_FOOT = 3;
        public static final int CLIMBER_UP = 4;
        public static final int CLIMBER_DN = 5;
        public static final int ABORT = 12;
     }

     public static final class ReefOperatorConstants{
        public static final int TWELVE_LEFT = 1;
        public static final int TWELVE_RIGHT = 2;
        public static final int TEN_LEFT = 3;
        public static final int TEN_RIGHT = 4;
        public static final int EIGHT_LEFT = 5;
        public static final int EIGHT_RIGHT = 6;
        public static final int SIX_LEFT = 7;
        public static final int SIX_RIGHT = 8;
        public static final int FOUR_LEFT = 9;
        public static final int FOUR_RIGHT = 10;
        public static final int TWO_LEFT = 11;
        public static final int TWO_RIGHT = 12;
     }

     public static final class ElevatorOperatorConstants{
        public static final int L4 = 1;
        public static final int RESET = 2;
        public static final int L3 = 3;
        public static final int INTAKE = 4;
        public static final int L2 = 5;
        public static final int REV = 6;
        public static final int L1 = 7;
        public static final int ALGAE = 8;
     }

     public static final class DistanceConstants{
        //Change later when installed
        public static final int leftAlignmentRangeId = 30;
        public static final int righttAlignmentRangeId = 31;

        public static final int TargetArray0 = 32;
        public static final int TargetArray1 = 33;
        public static final int TargetArray2 = 34;
        public static final int TargetArray3 = 35;
        public static final int TargetArray4 = 36;

        //Change later when installed
        public static final int alignmentDetectionRange = 0;
        public static final int poleDetectionRange = 0;

        public static final double reefScoringDistance = 0.4;//0.63;//0.4;//0.5; //0.4; //0.22;//0.32; //0.24 No lean //0.4; Lean
        public static final double reefDetectionThreshold = 0.8; //0.8;//0.75;
        //public static final double reefGuessThreshhold = 0.6;
        public static final double reefAlignedDistance = .23;
        public static final double maxTimer = 2.0;
        public static final double maxSpeed = 0.25;
     }

     public static final class CANdleConstants{
        public static final int CANdleID = 21;
        public static final int MaxBrightnessAngle = 90;
        public static final int MidBrightnessAngle = 180;
        public static final int ZeroBrightnessAngle = 270;

     }
 }
 