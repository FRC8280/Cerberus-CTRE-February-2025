// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Queue;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.AlgaeClimberOperatorConstants;
import frc.robot.commands.AlgeaArmCommand;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.IntakeReverse;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlgaeArm;

public class RobotContainer {
        int detectedAprilTag = -1;
        public final Vision[] vision = new Vision[Constants.Vision.CamNames.length];
        public final AlgaeArm m_Algae = new AlgaeArm();
        //public final Climber m_Climber = new Climber();
        public final Elevator m_Elevator = new Elevator();
        public final Effector m_Effector = new Effector();
        public ReefTargets m_ReefTargets = new ReefTargets();
        public double m_SlowSpeedMod = 1;

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity
        private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                                     
        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        // private final SwerveRequest.PointWheelsAt point = new
        // SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driverController = new CommandXboxController(0);
        // private final CommandXboxController operator = new CommandXboxController(1);

        private final Joystick CoralOperator = new Joystick(1);
        private final Joystick AlgeaAndClimberOperator = new Joystick(2);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


        /* Path follower */
        private final SendableChooser<Command> autoChooser;
        public Command automaticPath = null;

        public RobotContainer() {

                // Register Named Commands
                NamedCommands.registerCommand("Score Coral", new InstantCommand(()->m_Effector.ScoreCoral()));
                NamedCommands.registerCommand("Intake Complete", new WaitUntilCommand (() -> m_Effector.IntakeComplete()));
                NamedCommands.registerCommand("Done Scoring", new WaitUntilCommand(() -> !m_Effector.Scoring()));
                NamedCommands.registerCommand("Intake", new InstantCommand(() -> m_Effector.autonIntake()));
                     
                NamedCommands.registerCommand("Stow Elevator", new InstantCommand(()->m_Elevator.Stow())); 
                NamedCommands.registerCommand("Level 1", new InstantCommand(()->m_Elevator.LevelOne())); 
                NamedCommands.registerCommand("Level 2", new InstantCommand(()->m_Elevator.LevelTwo())); 
                NamedCommands.registerCommand("Level 3", new InstantCommand(()->m_Elevator.LevelThree())); 
                NamedCommands.registerCommand("Elevator Level 4", new InstantCommand(() -> m_Elevator.LevelFour()));
                NamedCommands.registerCommand("Reached Set State", new WaitUntilCommand(() -> m_Elevator.reachedSetState()));
                NamedCommands.registerCommand("Reset Elevator Zero", new InstantCommand(() ->m_Elevator.RunCurrentZeroing()));
                
                NamedCommands.registerCommand("Algae CheckPoint", new InstantCommand(() -> m_Elevator.AlgaeCheckpoint()));
                NamedCommands.registerCommand("Move Algae Arm", new InstantCommand(() -> m_Effector.MoveAlgeaArm()));
                NamedCommands.registerCommand("Algae Low", new InstantCommand(() -> m_Elevator.AlgeaLow()));
                NamedCommands.registerCommand("Effector Stop", new InstantCommand(() -> m_Effector.Stop()));
                NamedCommands.registerCommand("Back Up", new InstantCommand(() -> this.backUp(0.5)));

                NamedCommands.registerCommand("STOPNOW", new InstantCommand(() -> m_Effector.StopNewArm()));
                // Initialize each element of the vision array
                for (int i = 0; i < vision.length; i++) {
                        vision[i] = new Vision(i);
                }
                autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Mode", autoChooser);

                configureBindings();
        }

        public void RobotInit()
        {
                //m_Elevator.RunCurrentZeroing();
        }

        public enum Alignment {
                LEFT,
                RIGHT,
                CENTER
        }

        private void alignToTarget(boolean left)
         {

                if (detectedAprilTag == -1) {
                        if (left) 
                                leftAlign();
                        else
                                rightAlign();
                        return;
                }
                else 
                {
                  if (detectedAprilTag == 6 || detectedAprilTag == 7 || detectedAprilTag == 8 ||
                    detectedAprilTag == 18 || detectedAprilTag == 17 || detectedAprilTag == 19) 
                    {
                        if (left) 
                                leftAlign();
                        else
                                rightAlign();
                        return;
                    }
                    else{
                        if (left) 
                                rightAlign();
                        else
                                leftAlign();
                        return;
                    }
                    
                }
                
        }
        private void leftAlign() {
                Pose2d currentPose = drivetrain.getState().Pose;
                Pose2d destinationPos = m_ReefTargets.leftTarget;
                // Pose2d destinationPos = new Pose2d(14.8, 4.355,
                // Rotation2d.fromDegrees(174.06));
                if (destinationPos == null)
                        return;

                // The rotation component in these poses represents the direction of travel
                Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());

                List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, destinationPos);
                PathPlannerPath path = new PathPlannerPath(
                                waypoints,
                                new PathConstraints(
                                                2.5, 2.5,
                                                Units.degreesToRadians(360), Units.degreesToRadians(540)),
                                null, // Ideal starting state can be null for on-the-fly paths
                                new GoalEndState(0.0, destinationPos.getRotation())// currentPose.getRotation())
                );

                // Prevent this path from being flipped on the red alliance, since the given
                // positions are already correct
                path.preventFlipping = true;
                automaticPath = AutoBuilder.followPath(path);
                automaticPath.schedule();
                // AutoBuilder.followPath(path).schedule();
        }

        private void rightAlign() {
                Pose2d currentPose = drivetrain.getState().Pose;
                Pose2d destinationPos = m_ReefTargets.rightTarget;
                // Pose2d destinationPos = new Pose2d(14.8, 3.4,Rotation2d.fromDegrees(174.06));
                if (destinationPos == null)
                        return;
                // The rotation component in these poses represents the direction of travel
                Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());

                List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, destinationPos);
                PathPlannerPath path = new PathPlannerPath(
                                waypoints,
                                new PathConstraints(
                                                Constants.AutoAlignment.maxVelocity,
                                                Constants.AutoAlignment.maxAcceleration,
                                                Constants.AutoAlignment.MaxAngularRate,
                                                Constants.AutoAlignment.MaxAngularAcceleration),
                                null, // Ideal starting state can be null for on-the-fly paths
                                new GoalEndState(0.0, destinationPos.getRotation())// currentPose.getRotation())
                );

                // Prevent this path from being flipped on the red alliance, since the given
                // positions are already correct
                path.preventFlipping = true;
                automaticPath = AutoBuilder.followPath(path);
                automaticPath.schedule();
        }

        private void backUp(double distance) {
                Pose2d currentPose = drivetrain.getState().Pose;

                //try to calculate
                
                double x = currentPose.getX();
                double y = currentPose.getY();
                Rotation2d heading = currentPose.getRotation();
                double newX = x - distance *Math.cos(heading.getRadians());
                double newY = y - distance*Math.sin(heading.getRadians());

                Pose2d destinationPos = new Pose2d(newX,newY,heading);

               /*Pose2d destinationPos = new Pose2d(
                                currentPose.getTranslation()
                                                .plus(new Translation2d(0.1524, 0).rotateBy(currentPose.getRotation())),
                                currentPose.getRotation()); */ 
                // Pose2d destinationPos = new Pose2d(14.8, 3.4,Rotation2d.fromDegrees(174.06));
                if (destinationPos == null)
                        return;
                // The rotation component in these poses represents the direction of travel
                Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());

                List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, destinationPos);
                PathPlannerPath path = new PathPlannerPath(
                                waypoints,
                                new PathConstraints(
                                                Constants.AutoAlignment.maxVelocity,
                                                Constants.AutoAlignment.maxAcceleration,
                                                Constants.AutoAlignment.MaxAngularRate,
                                                Constants.AutoAlignment.MaxAngularAcceleration),
                                null, // Ideal starting state can be null for on-the-fly paths
                                new GoalEndState(0.0, destinationPos.getRotation())// currentPose.getRotation())
                );

                // Prevent this path from being flipped on the red alliance, since the given
                // positions are already correct
                path.preventFlipping = true;
                automaticPath = AutoBuilder.followPath(path);
                automaticPath.schedule();
        }


        private void CancelAutomaticMovement() {
                if ((automaticPath == null) || (!automaticPath.isScheduled()))
                        return;
                automaticPath.cancel();
                automaticPath = null;
        }

        private boolean autoPathActive() {
                return (automaticPath != null) && (automaticPath.isScheduled());
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.

                // Driver controls
                // Slow motion
                new Trigger(() -> driverController.getLeftTriggerAxis() > 0.25)
                                .whileTrue(new InstantCommand(() -> m_SlowSpeedMod = 0.5))
                                .onFalse(new InstantCommand(() -> m_SlowSpeedMod = 1));
                
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                        drivetrain.applyRequest(
                                        () -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed * m_SlowSpeedMod) // Drive forward with negative Y (forward)
                                        .withVelocityY(-driverController.getLeftX() * MaxSpeed * m_SlowSpeedMod) // Drive left with negative X (left)
                                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate * m_SlowSpeedMod)) // Drivecounterclockwise with negative X (left)
                                        ); 
                        
                // reset the field-centric heading on left bumper press
                driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Braking
                new Trigger(() -> driverController.getRightTriggerAxis() > 0.25)
                                .whileTrue(drivetrain.applyRequest(() -> brake));

                // Binding to call CancelAutomaticMovement() if any analog stick is over 50%
                new Trigger(() -> Math.abs(driverController.getLeftX()) > 0.4 ||
                                Math.abs(driverController.getLeftY()) > 0.4 ||
                                Math.abs(driverController.getRightX()) > 0.4 ||
                                Math.abs(driverController.getRightY()) > 0.4)
                                .onTrue(new InstantCommand(() -> this.CancelAutomaticMovement())
                                        .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))
                                        .andThen(new InstantCommand(() -> m_Elevator.Stow()))
                                        .andThen(new InstantCommand(() -> m_Effector.Stop()))
                                        );

                driverController.pov(0)
                                .whileTrue(drivetrain.applyRequest(
                                                () -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
                driverController.pov(180)
                                .whileTrue(drivetrain.applyRequest(
                                                () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                driverController.back().and(driverController.y())
                                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                driverController.back().and(driverController.x())
                                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                driverController.start().and(driverController.y())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                driverController.start().and(driverController.x())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // operator code
                // New trigger for when the elevator is atStowed() is true but ElevatorAtZero()
                // is false
                /*new Trigger(() -> m_Elevator.atStowed() && !m_Elevator.ElevatorAtZero())
                                .onTrue(new InstantCommand(() -> System.out.println("Elevator is at stowed position but not at zero."))
                                //.andThen(m_Elevator.RunCurrentZeroing())
                                );*/

                // Winch Code
               /*  new JoystickButton(AlgeaAndClimberOperator, Constants.AlgaeClimberOperatorConstants.CLIMBER_BUTTON_DN)
                                .whileTrue(new ClimberDownCommand(m_Climber));
                new JoystickButton(AlgeaAndClimberOperator, Constants.AlgaeClimberOperatorConstants.CLIMBER_BUTTON_UP)
                                .whileTrue(new ClimberUpCommand(m_Climber));*/

                // Bindings to control Elevator Level, wait until it aligns, then runs
                // sequential command gorup to score
                new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_LL4)
                                .onTrue(new InstantCommand(() -> alignToTarget(true))
                                                .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new InstantCommand(() -> m_Elevator.LevelFour()))
                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new InstantCommand(() -> m_Effector.ScoreCoral()))
                                                .andThen(new WaitUntilCommand(() -> !m_Effector.Scoring()))
                                                .andThen(new WaitCommand(0.15))
                                                .andThen(new InstantCommand(() -> m_Elevator.Stow()))

                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new WaitCommand(0.6))
                                                .andThen(m_Elevator.RunCurrentZeroing()) // Todo make a proper reverse.
                                                
                                                );
                new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_RL4)
                                .onTrue(new InstantCommand(() -> alignToTarget(false))
                                                .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new InstantCommand(() -> m_Elevator.LevelFour()))
                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new InstantCommand(() -> m_Effector.ScoreCoral()))
                                                .andThen(new WaitUntilCommand(() -> !m_Effector.Scoring()))
                                                .andThen(new WaitCommand(0.15))
                                                .andThen(new InstantCommand(() -> m_Elevator.Stow()))

                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new WaitCommand(0.6))
                                                .andThen(m_Elevator.RunCurrentZeroing()) // Todo make a proper reverse.
                                                
                                                );
                new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_LL3)
                                .onTrue(new InstantCommand(() -> alignToTarget(true))
                                                .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new InstantCommand(() -> m_Elevator.LevelThree()))
                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new InstantCommand(() -> m_Effector.ScoreCoral()))
                                                .andThen(new WaitUntilCommand(() -> !m_Effector.Scoring()))
                                                .andThen(new WaitCommand(0.15))
                                                .andThen(new InstantCommand(() -> m_Elevator.Stow()))
                                                
                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new WaitCommand(0.6))
                                                .andThen(m_Elevator.RunCurrentZeroing()) // Todo make a proper reverse.
                                                 
                                                );
                new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_RL3)
                                .onTrue(new InstantCommand(() -> alignToTarget(false))
                                                .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new InstantCommand(() -> m_Elevator.LevelThree()))
                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new InstantCommand(() -> m_Effector.ScoreCoral()))
                                                .andThen(new WaitUntilCommand(() -> !m_Effector.Scoring()))
                                                .andThen(new WaitCommand(0.15))
                                                .andThen(new InstantCommand(() -> m_Elevator.Stow()))

                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new WaitCommand(0.6))
                                                .andThen(m_Elevator.RunCurrentZeroing()) // Todo make a proper reverse.
                                                
                                                );
                new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_LL2)
                                .onTrue(new InstantCommand(() -> alignToTarget(true))
                                                .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new InstantCommand(() -> m_Elevator.LevelTwo()))
                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new InstantCommand(() -> m_Effector.ScoreCoral()))
                                                .andThen(new WaitUntilCommand(() -> !m_Effector.Scoring()))
                                                .andThen(new WaitCommand(0.15))
                                                .andThen(new InstantCommand(() -> m_Elevator.Stow()))
                                                
                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new WaitCommand(0.6))
                                                .andThen(m_Elevator.RunCurrentZeroing()) // Todo make a proper reverse.
                                                
                                                );
                new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_RL2)
                                .onTrue(new InstantCommand(() -> alignToTarget(false))
                                                .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new InstantCommand(() -> m_Elevator.LevelTwo()))
                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new InstantCommand(() -> m_Effector.ScoreCoral()))
                                                .andThen(new WaitUntilCommand(() -> !m_Effector.Scoring()))
                                                .andThen(new WaitCommand(0.15))
                                                .andThen(new InstantCommand(() -> m_Elevator.Stow()))
                                                
                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new WaitCommand(0.6))
                                                .andThen(m_Elevator.RunCurrentZeroing()) // Todo make a proper reverse.
                                                
                                                );

                 new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_L1)
                                                .onTrue(new InstantCommand(() -> m_Elevator.LevelOne())
                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new InstantCommand(() -> m_Effector.ScoreCoral()))
                                                .andThen(new WaitUntilCommand(() -> !m_Effector.Scoring()))
                                                .andThen(new WaitCommand(0.15))
                                                .andThen(new InstantCommand(() -> m_Elevator.Stow()))
                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                                .andThen(new WaitCommand(0.6))
                                                .andThen(m_Elevator.RunCurrentZeroing()) // Todo make a proper reverse.
                                                );

                // New trigger to call UpLevel once when the vertical joystick value is over 25%
                new Trigger(() -> CoralOperator.getY() >= 0.75)
                                .onTrue(m_Elevator.runOnce(() -> m_Elevator.UpLevel()));

                // New trigger to call UpLevel once when the vertical joystick value is over 25%
                new Trigger(() -> CoralOperator.getY() < -0.75)
                                .onTrue(m_Elevator.runOnce(() -> m_Elevator.DownLevel()));

                // New trigger to call UpLevel once when the vertical joystick value is over 25%
                new Trigger(() -> CoralOperator.getX() > 0.75)
                                .onTrue(m_Elevator.runOnce(() -> m_Elevator.IncrementIncrease()));

                // New trigger to call UpLevel once when the vertical joystick value is over 25%
                new Trigger(() -> CoralOperator.getX() < -0.75)
                                .onTrue(m_Elevator.runOnce(() -> m_Elevator.IncrementDecrease()));

                

                new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.RESET_BUTTON)
                                .onTrue(new ParallelCommandGroup(
                                                                m_Elevator.RunCurrentZeroing(),
                                                                new InstantCommand(() -> m_Effector.Stop())));

                new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.MANUAL_BUTTON)
                                .onTrue(new InstantCommand(() -> m_Effector.ScoreCoral()));

                new JoystickButton(AlgeaAndClimberOperator, Constants.AlgaeClimberOperatorConstants.CLEAR_HIGH)
                                .onTrue(new InstantCommand(() -> m_Elevator.AlgeaHigh())
                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                .andThen(new InstantCommand(() -> m_Effector.MoveAlgeaArm())
                                .andThen(new WaitCommand(0.5))
                                .andThen(new InstantCommand(()->m_Effector.Stop()))
                                .andThen(new InstantCommand(() -> this.backUp(0.5)))
                                .andThen(new InstantCommand(() -> m_Elevator.Stow()))
                                ));
                new JoystickButton(AlgeaAndClimberOperator, Constants.AlgaeClimberOperatorConstants.CLEAR_LOW)
                                .onTrue(new InstantCommand(() -> m_Elevator.AlgaeCheckpoint())
                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                .andThen(new InstantCommand(() -> m_Effector.MoveAlgeaArm())
                                .andThen(new InstantCommand(() -> m_Elevator.AlgeaLow())
                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                .andThen(new WaitCommand(0.5))
                                .andThen(new InstantCommand(()->m_Effector.Stop()))
                                .andThen(new InstantCommand(() -> this.backUp(0.5)))
                                .andThen(new InstantCommand(() -> m_Elevator.Stow())))
                                ));
                        new JoystickButton(AlgeaAndClimberOperator, Constants.AlgaeClimberOperatorConstants.ALGAE_EJECT)
                                .onTrue(new InstantCommand(()-> backUp(0.5)));

                //.onTrue(new InstantCommand(() -> m_Effector.ResetAlgeaArm()));

                // Turn on coral intake with left button press
                new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_INTAKE_BUTTON)
                                .onTrue(new InstantCommand(() -> m_Elevator.Stow())
                                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()).withTimeout(0.5))
                                                .andThen(new WaitCommand(0.1))
                                                .andThen(new InstantCommand(() -> m_Elevator.RunCurrentZeroing()))
                                                .andThen(new InstantCommand(() -> m_Effector.IntakeCoral())));

                new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.REVERSE_INTAKE)
                                        .whileTrue(new IntakeReverse(m_Effector));

                new JoystickButton(AlgeaAndClimberOperator, Constants.AlgaeClimberOperatorConstants.L_IN)
                                        .whileTrue(new AlgeaArmCommand(m_Algae));

                /*JoystickButton coralIntakeButton = new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_INTAKE_BUTTON);
                
                coralIntakeButton.
                coralIntakeButton.whenPressed(new InstantCommand(() -> m_effector.intake()))
                                        .whileHeld(new InstantCommand(() -> m_effector.EjectCoral()))
                                        .whenReleased(new InstantCommand(() -> m_effector.stop()));*/

                // .whileTrue(new InstantCommand(() -> System.out.println("Button 8 pressed")));


                // operator.getLeftY().whileActiveContinuous(() -> m_Elevator.SetPower());
                //
                // operator.x().onTrue(new InstantCommand(() -> m_Elevator.SetPower((float)
                // 0.0)));
                // operator.y().onTrue(new InstantCommand(() ->
                // m_Elevator.SetPower((float)-0.15)));

                // drivetrain.registerTelemetry(logger::telemeterize);
        }

        private SequentialCommandGroup AlgeaLowSequence() {
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_Elevator.AlgaeCheckpoint()),
                    new WaitUntilCommand(() -> m_Elevator.reachedSetState()),
                    new InstantCommand(() -> m_Effector.MoveAlgeaArm()),
                    new InstantCommand(() -> m_Elevator.AlgeaLow()),
                    new WaitUntilCommand(() -> m_Elevator.reachedSetState()),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> m_Effector.Stop()),
                    new InstantCommand(() -> this.backUp(0.5)),
                    new InstantCommand(() -> m_Elevator.Stow())
                );
            }

        public void Periodic() {
                // SmartDashboard.putNumber("Operator Joystick Y", CoralOperator.getY());
                // SmartDashboard.putNumber("Operator Joystick X",CoralOperator.getX());
        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return autoChooser.getSelected();
        }
}
