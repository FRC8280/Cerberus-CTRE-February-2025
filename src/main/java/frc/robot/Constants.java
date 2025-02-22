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
     public static class Vision {

        public static double CAMERA_HEIGHT_METERS = Units.inchesToMeters(11.5);
        public static double TARGET_HEIGHT_METERS = Units.inchesToMeters(51.875);
        public static double CAMERA_PITCH_RADIANS = Units.degreesToRadians(45);

        public static final String[] CamNames = {
            "FrontRightCamera", "FrontLeftCamera"
        };

       /* // example Cam mounted facing forward, half a meter forward of center, half a meter up from center*/
        public static final Transform3d[] cameraTransforms = {
            new Transform3d(new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(2.5), Units.inchesToMeters(11.5)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))),
            new Transform3d(new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(2.5), Units.inchesToMeters(11.5)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))
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

        public static final double levelOne = 2.03564453125; //idk find the values later
        public static final double levelTwo = 6.96728515625;
        public static final double levelThree = 14.49;//14.0595703125;
        public static final double levelFour = 25.45;
        public static final double kStowed = 0;

        public static final int kCurrentLimit = 60;

        public static final double kElevatorDetectionRange = 450;
        
    }

    public static final class Effector{
        public static final int kMotorCanId = 16;
        public static final int kAlgeaMotorCandID = 17;
        public static final double kCurrentLimit = 60;
        public static final double kVoltageComp = 12;
        public static final double kSpeed = 50;
        
        public static final int kFrontLaserCanId = 40;
        public static final int kRearLaserCanId = 39;
        public static final int kCoralDetectionRange = 70;
        public static final int intakeTimerMax = 3;
        public static final double intakeSpeed = 2;//50;//300; //0.5;
        public static final int ejectSpeed = -10;
        public static final int scoreSpeed = 10;

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
         public static final double k_intakeSpeed = 0;
         public static final double k_EjectSpeed = 0;
        
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
         public static final double kIntakePosition = 0.0;
         public static final double kScoringPosition = 0.0;
 
         public static final double kSoftLimitReverse = 0; //-140;
         public static final double kSoftLimitForward = 4.77;//4.6 or 143.878
 
         public static final double kArmZeroCosineOffset = - Math.PI / 6; //radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
 
         public static final double kIntakeBarDistanceMM = 60;
     }

     public static final class CoralOperatorConstants{
        public static final int CORAL_LL4 = 1;
        public static final int CORAL_RL4 = 2;
        public static final int CORAL_LL3 = 3;
        public static final int CORAL_RL3 = 4;
        public static final int CORAL_LL2 = 5;
        public static final int CORAL_RL2 = 6;
        public static final int CORAL_L1 = 7;
        public static final int MANUAL_BUTTON = 8;
        public static final int CORAL_INTAKE_BUTTON = 10;
     }

     public static final class AlgaeClimberOperatorConstants{
        public static final int CLIMBER_BUTTON_UP = 6;
        public static final int CLIMBER_BUTTON_DN = 7;
     }
 }
 