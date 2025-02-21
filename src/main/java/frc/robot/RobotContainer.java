// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Robot;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae;

import frc.robot.Vision;

public class RobotContainer {
    public Vision vision;
    public final Algae m_Algae = new Algae();
    public final Climber m_Climber = new Climber();
    public final Elevator m_Elevator = new Elevator();
    public final Effector m_Effector = new Effector();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    //private final CommandXboxController operator = new CommandXboxController(1);

    private final Joystick CoralOperator = new Joystick(1);
    private final Joystick AlgeaAndClimberOperator = new Joystick(2);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    public Command automaticPath = null; 
    
    public RobotContainer() {
        //vision = new Vision();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }
    

    private void leftAlign()
    {
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d destinationPos = vision.leftTarget;
        //Pose2d destinationPos = new Pose2d(14.8, 4.355, Rotation2d.fromDegrees(174.06));
        if(destinationPos== null)
         return;

        //Pose2d destinationPos = new Pose2d(14.6, 4.24, new Rotation2d(Units.degreesToRadians(-177.8)));
        // The rotation component in these poses represents the direction of travel
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
    
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, destinationPos);
        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            new PathConstraints(
            2.5, 2.5, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
            ),
            null, // Ideal starting state can be null for on-the-fly paths
            new GoalEndState(0.0, destinationPos.getRotation())//currentPose.getRotation())
        );

        // Prevent this path from being flipped on the red alliance, since the given positions are already correct
        path.preventFlipping = true;
        automaticPath = AutoBuilder.followPath(path);
        automaticPath.schedule();
        //AutoBuilder.followPath(path).schedule();
    }

    private void rightAlign()
    {
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d destinationPos = vision.rightTarget;
        //Pose2d destinationPos = new Pose2d(14.8, 3.4, Rotation2d.fromDegrees(174.06));
        if(destinationPos== null)
            return;
        //Pose2d destinationPos = new Pose2d(14.6, 4.24, new Rotation2d(Units.degreesToRadians(-177.8)));
        // The rotation component in these poses represents the direction of travel
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
    
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, destinationPos);
        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            new PathConstraints(
            2.5, 2.5, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
            ),
            null, // Ideal starting state can be null for on-the-fly paths
            new GoalEndState(0.0, destinationPos.getRotation())//currentPose.getRotation())
        );

        // Prevent this path from being flipped on the red alliance, since the given positions are already correct
        path.preventFlipping = true;
        automaticPath = AutoBuilder.followPath(path);
        automaticPath.schedule();
        //AutoBuilder.followPath(path).schedule();
    }


    private void CancelAutomaticMovement()
    {
        if( (automaticPath == null) || (!automaticPath.isScheduled()))
            return;
        automaticPath.cancel();
        automaticPath = null;
    }
    Command AltMovementTest()
    {
        Pose2d destinationPos = new Pose2d(14.6, 4.24, new Rotation2d(Units.degreesToRadians(-177.8)));
        PathConstraints constraints = new PathConstraints(
            2.5, 2.5, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)
          );
          return AutoBuilder.pathfindToPose(destinationPos, constraints, 0.0); 
    }
    Command AltMovementTest2()
    {
        Pose2d currentPose = drivetrain.getState().Pose;
    
        
        Pose2d destinationPos = new Pose2d(14.6, 4.24, new Rotation2d(Units.degreesToRadians(-177.8)));
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
      //Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());
    
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, destinationPos);
      PathPlannerPath path = new PathPlannerPath(
        waypoints, 
        new PathConstraints(
          2.5, 2.5, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),
        null, // Ideal starting state can be null for on-the-fly paths
        new GoalEndState(0.0, destinationPos.getRotation())//currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;
      return(AutoBuilder.followPath(path));
      
    }

    void killAutoMovement()
    {
        if((automaticPath != null) && (automaticPath.isScheduled()))
            automaticPath.end(true);
    }
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        //joystick.x().whileTrue(AltMovementTest());
          //operator.y().onTrue(new InstantCommand(()->this.leftAlign()));
      //  operator.a().onTrue(new InstantCommand(() -> this.rightAlign()));
        joystick.x().onTrue(new InstantCommand(()->this.CancelAutomaticMovement()));

        /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Winch Code
        new JoystickButton(AlgeaAndClimberOperator, Constants.AlgaeClimberOperatorConstants.CLIMBER_BUTTON_DN)
            .whileTrue(new ClimberDownCommand(m_Climber));
        new JoystickButton(AlgeaAndClimberOperator, Constants.AlgaeClimberOperatorConstants.CLIMBER_BUTTON_UP)
            .whileTrue(new ClimberUpCommand(m_Climber));
            
        //new JoystickButton(AlgeaAndClimberOperator, Constants.ClimberConstants.CLIMBER_BUTTON_UP)
        //.whileTrue(new InstantCommand(() -> System.out.println("Button 6 pressed")));


        //AlgeaAndClimberOperator.getRawButton(6).whileTrue(new ClimberUpCommand(m_climber));
        //operator.pov(180).whileTrue(new ClimberDownCommand(m_climber));

        //Turn on coral intake with left button press
        new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_INTAKE_BUTTON)
        .onTrue(new InstantCommand(() -> m_Elevator.Stow())
        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState())));


        //Because Dr. Lapetina says write pseudo code
        //Bindings to control Elevator Level, wait until it reaches the setpoint, then wait a quarter second, then score coral
        new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_LL4)
        .onTrue(new InstantCommand(() -> m_Elevator.LevelFour())
        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
        .andThen(new WaitCommand(0.1))
        .andThen(new InstantCommand(() -> m_Effector.ScoreCoral()))
        .andThen(new WaitUntilCommand(() -> !m_Effector.Scoring()))
        .andThen(new WaitCommand(0.15))
        .andThen(new InstantCommand(() -> m_Elevator.Stow()))
        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
        .andThen(new WaitCommand(0.25))
        .andThen(m_Elevator.RunCurrentZeroing())
        );

        new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_RL4)
        .onTrue(new InstantCommand(() -> m_Elevator.LevelFour())
        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
        .andThen(new WaitCommand(0.25))
        .andThen(new InstantCommand(() -> m_Effector.ScoreCoral())));
        
        new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_LL3)
        .onTrue(new InstantCommand(() -> m_Elevator.LevelThree())
        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
        .andThen(new WaitCommand(0.1))
        .andThen(new InstantCommand(() -> m_Effector.ScoreCoral()))
        .andThen(new WaitUntilCommand(() -> !m_Effector.Scoring())) ////Added 
        .andThen(new WaitCommand(0.15))   //Added 
        .andThen(new InstantCommand(() -> m_Elevator.Stow()))
        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState())) //Added
        .andThen(new WaitCommand(0.25))
        .andThen(m_Elevator.RunCurrentZeroing()) //Added
        );

        new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_RL3)
        .onTrue(new InstantCommand(() -> m_Elevator.LevelThree())
        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
        .andThen(new WaitCommand(0.25))
        .andThen(new InstantCommand(() -> m_Effector.ScoreCoral())));

        new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_LL2)
        .onTrue(new InstantCommand(() -> m_Elevator.LevelTwo())
        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
        .andThen(new WaitCommand(0.1))
        .andThen(new InstantCommand(() -> m_Effector.ScoreCoral()))
        .andThen(new WaitUntilCommand(() -> !m_Effector.Scoring()))
        .andThen(new WaitCommand(0.15))
        .andThen(new InstantCommand(() -> m_Elevator.Stow()))
        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
        .andThen(new WaitCommand(0.25))
        .andThen(m_Elevator.RunCurrentZeroing())
        );
        
        new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_RL2)
        .onTrue(new InstantCommand(() -> m_Elevator.LevelTwo())
        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
        .andThen(new WaitCommand(0.25))
        .andThen(new InstantCommand(() -> m_Effector.ScoreCoral())));


        new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.CORAL_L1)
        .onTrue(new InstantCommand(() -> m_Elevator.LevelOne())
        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
        .andThen(new WaitCommand(0.25))
        .andThen(new InstantCommand(() -> m_Effector.ScoreCoral())));

        new JoystickButton(CoralOperator, Constants.CoralOperatorConstants.MANUAL_BUTTON)
        .onTrue(m_Elevator.RunCurrentZeroing());
        //.whileTrue(new InstantCommand(() -> System.out.println("Button 8 pressed")));

       // operator.pov(0).whileTrue(new InstantCommand(() -> m_Elevator.IncrementIncrease()));
        //operator.pov(180).whileTrue(new InstantCommand(() -> m_Elevator.IncrementDecrease()));
         
        
        //operator.getLeftY().whileActiveContinuous(() -> m_Elevator.SetPower());
        

        //operator.x().onTrue(new InstantCommand(() -> m_Elevator.SetPower((float) 0.0)));
        //operator.y().onTrue(new InstantCommand(() -> m_Elevator.SetPower((float)-0.15)));

        drivetrain.registerTelemetry(logger::telemeterize);
        //AutoBuilder
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }


}
