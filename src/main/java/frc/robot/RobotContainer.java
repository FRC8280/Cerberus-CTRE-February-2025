// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.commands.IntakeReverse;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DistanceSensorSystem;
import frc.robot.subsystems.DistanceSensorSystem.ReefPoleAlignment;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;

public class RobotContainer {

    enum AlgeaHeight {
        Low,
        Hi
    };

    private Timer m_cancelAutomovementTimer = new Timer();
    private boolean m_scoreCommandAborted = false;
    private boolean m_algeaScoreCommandAborted = false;

    public final Vision[] vision = new Vision[Constants.Vision.CamNames.length];
    // public final AlgaeArm m_Algae = new AlgaeArm();
    public final Climber m_Climber = new Climber();
    public final Elevator m_Elevator = new Elevator();
    public final Effector m_Effector = new Effector();
    // private final CANdleSystem m_candleSubsystem = new CANdleSystem();
    public ReefTargets m_ReefTargets = new ReefTargets();
    public DistanceSensorSystem m_DistanceSensorSystem = new DistanceSensorSystem();
    public double m_SlowSpeedMod = 1;
    public boolean m_AutoAlignOff = false;
    public boolean m_ArmedClimber = false;
    public boolean m_RunScoring = false;
    public boolean m_AlgeaScoring = false;
    public boolean m_L1Scoring = false;

    public double m_ElevatorDestination = Constants.Elevator.noDestination;
    public int m_selectedReef = Constants.AutoAlignment.noReef;
    public boolean m_AlgeaButtonPressed = false;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                  // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                      // second
                                                                                      // max angular velocity
    /*
     * private final SwerveRequest.RobotCentric robotCentricDrive = new
     * SwerveRequest.RobotCentric()
     * .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop
     * control for drive
     */

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

    private final Joystick ManualOperator = new Joystick(1);
    private final Joystick ReefOperator = new Joystick(2);
    private final Joystick ElevatorOperator = new Joystick(3);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    public Command automaticPath = null;

    public static Command threadCommand() {
        return Commands.sequence(
                Commands.waitSeconds(20),
                Commands.runOnce(() -> Threads.setCurrentThreadPriority(true, 10))).ignoringDisable(true);
    }

    public RobotContainer() {

        // Register Named Commands
        NamedCommands.registerCommand("Intake Complete", new WaitUntilCommand(() -> m_Effector.IntakeComplete()));
        NamedCommands.registerCommand("Done Scoring", new WaitUntilCommand(() -> !m_Effector.Scoring()));
        NamedCommands.registerCommand("Intake", new InstantCommand(() -> m_Effector.autonIntake()));

        NamedCommands.registerCommand("Stow Elevator", new InstantCommand(() -> m_Elevator.Stow()));
        NamedCommands.registerCommand("Set Stage 1",
                new InstantCommand(() -> SetElevatorDestination(Constants.Elevator.levelOne)));
        NamedCommands.registerCommand("Set Stage 2",
                new InstantCommand(() -> SetElevatorDestination(Constants.Elevator.levelTwo)));
        NamedCommands.registerCommand("Set Stage 3",
                new InstantCommand(() -> SetElevatorDestination(Constants.Elevator.levelThree)));
        NamedCommands.registerCommand("Set Stage 4",
                new InstantCommand(() -> SetElevatorDestination(Constants.Elevator.levelFour)));
        NamedCommands.registerCommand("Set Score Trigger", new InstantCommand(() -> SetScoreTrigger(true)));
        NamedCommands.registerCommand("Sequence Complete", new WaitUntilCommand(() -> GetScoreTriggerFlag()));

        NamedCommands.registerCommand("Level 1", new InstantCommand(() -> m_Elevator.LevelOne()));
        NamedCommands.registerCommand("Level 2", new InstantCommand(() -> m_Elevator.LevelTwo()));
        NamedCommands.registerCommand("Level 3", new InstantCommand(() -> m_Elevator.LevelThree()));
        NamedCommands.registerCommand("Elevator Level 4", new InstantCommand(() -> m_Elevator.LevelFour()));
        // NamedCommands.registerCommand("Set Score Trigger", new
        // InstantCommand(()->SetScoreTrigger(true)));

        NamedCommands.registerCommand("Reached Set State", new WaitUntilCommand(() -> m_Elevator.reachedSetState()));
        NamedCommands.registerCommand("Reset Elevator Zero", new InstantCommand(() -> m_Elevator.RunCurrentZeroing()));

        NamedCommands.registerCommand("Algae CheckPoint", new InstantCommand(() -> m_Elevator.AlgaeCheckpoint()));
        NamedCommands.registerCommand("Move Algae Arm", new InstantCommand(() -> m_Effector.MoveAlgeaArm()));
        NamedCommands.registerCommand("Algae Low", new InstantCommand(() -> m_Elevator.AlgeaLow()));
        NamedCommands.registerCommand("Effector Stop", new InstantCommand(() -> m_Effector.Stop()));
        NamedCommands.registerCommand("Back Up", new InstantCommand(() -> this.backUp(0.5)));

        NamedCommands.registerCommand("STOPNOW", new InstantCommand(() -> m_Effector.StopNewArm()));

        NamedCommands.registerCommand("Algea CheckPoint", new WaitCommand(0.15));
        NamedCommands.registerCommand("Score Coral", new InstantCommand(() -> m_Effector.ScoreCoral()));

        // Initialize each element of the vision array
        for (int i = 0; i < vision.length; i++) {
            vision[i] = new Vision(i);
        }
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
        // m_candleSubsystem.m_candle.setLEDs(255, 51, 51, 0, 0, 4);
    }

    public void EnableManualMode()
    {
        m_AutoAlignOff = true;
        System.out.println("Enabling Manual Mode");
    }
    public void DisableManualMode(){
        m_AutoAlignOff = false;
        System.out.println("Disable Manual Mode");
    }

    public void ArmClimber() {
        m_ArmedClimber = true;
        m_Climber.ArmClimber();
    }
    public void DisarmClimber(){
        m_ArmedClimber = false;
    }
    public boolean ManualModeEngaged() {
        return m_AutoAlignOff;
    }

    public void NewScoreAttempt() {
        m_scoreCommandAborted = false;
    }

    public void NewAlgeaScoreAttempt() {
        m_algeaScoreCommandAborted = false;
    }

    public boolean CommandAborted() {
        return m_scoreCommandAborted;
    }

    public boolean AlgeaCommandAborted() {
        return m_algeaScoreCommandAborted;
    }

    public boolean GetScoreTrigger() {
        return m_RunScoring && !CommandAborted();
    }

    public boolean GetAlgeaScoreTrigger() {
        return m_AlgeaScoring && !CommandAborted();
    }

    public void SetAlgeaScoreFlag(boolean value) {
        m_AlgeaScoring = value;
    }

    public boolean GetScoreTriggerFlag() {
        return m_RunScoring;
    }

    public void SetScoreTrigger(boolean value) {
        m_RunScoring = value;
    }

    public void SetElevatorDestination(double destination) {
        m_ElevatorDestination = destination;
    }

    public double GetElevatorDestination() {
        return m_ElevatorDestination;
    }

    public boolean ElevatorHasDestination() {
        return m_ElevatorDestination != Constants.Elevator.noDestination;
    }

    public void resetElevatorDestination() {
        m_ElevatorDestination = Constants.Elevator.noDestination;
    }

    public void resetElevatorAndBranch() {
        m_ElevatorDestination = Constants.Elevator.noDestination;
        m_selectedReef = Constants.AutoAlignment.noReef;
    }

    public void resetReefBranch() {
        m_selectedReef = Constants.AutoAlignment.noReef;
    }

    public void recordAlgeaButtonPressed() {
        m_AlgeaButtonPressed = true;
    }

    public boolean getAlgeaButtonPressed() {
        return m_AlgeaButtonPressed;
    }

    public void disableAlgeaButtonPressed() {
        m_AlgeaButtonPressed = false;
    }

    public void RobotInit() {
        System.out.println("Initializing ROBOT LOOK AT ME");
        m_Climber.ResetClimber();
        // m_Elevator.RunCurrentZeroing();
    }

    public enum Alignment {
        LEFT,
        RIGHT,
        CENTER
    }

    public boolean DriverInterrupt() {

        if (m_cancelAutomovementTimer.isRunning() && !m_cancelAutomovementTimer.hasElapsed(2))
            return false;
        else if (m_cancelAutomovementTimer.isRunning() && m_cancelAutomovementTimer.hasElapsed(2))
            m_cancelAutomovementTimer.stop();

        boolean abort = ManualOperator.getRawButtonPressed(Constants.ManualOperatorConstants.ABORT) ||
                Math.abs(driverController.getLeftX()) > 0.4 ||
                Math.abs(driverController.getLeftY()) > 0.4 ||
                Math.abs(driverController.getRightX()) > 0.4;

        if (abort) {
            m_cancelAutomovementTimer.reset();
            m_cancelAutomovementTimer.start();
        }

        return abort;
    }

    SwerveRequest.RobotCentric stopRobotMovementRequest = new SwerveRequest.RobotCentric();

    private SwerveRequest StopRobotNow() {

        stopRobotMovementRequest.VelocityY = 0;
        return stopRobotMovementRequest;
    }

    SwerveRequest.RobotCentric alignShotRobotRequest = new SwerveRequest.RobotCentric();

    private SwerveRequest AlignShotRequest() {

        ReefPoleAlignment currentAlignment = m_DistanceSensorSystem.LocateReefPole();

        if (currentAlignment == ReefPoleAlignment.FAR_LEFT)
            alignShotRobotRequest.VelocityY = -Constants.AutoAlignment.AutoAlignmentSpeed;
        else if (currentAlignment == ReefPoleAlignment.LEFT)
            alignShotRobotRequest.VelocityY = -Constants.AutoAlignment.AutoAlignmentSpeed;
        else if (currentAlignment == ReefPoleAlignment.RIGHT)
            alignShotRobotRequest.VelocityY = +Constants.AutoAlignment.AutoAlignmentSpeed;
        else if (currentAlignment == ReefPoleAlignment.FAR_RIGHT)
            alignShotRobotRequest.VelocityY = +Constants.AutoAlignment.AutoAlignmentSpeed;
        else if (currentAlignment == ReefPoleAlignment.CENTER)
            alignShotRobotRequest.VelocityY = 0;
        else
            alignShotRobotRequest.VelocityY = 0;

        return alignShotRobotRequest;
    }

    SwerveRequest.RobotCentric alignReefEdgeRequest = new SwerveRequest.RobotCentric();

    private SwerveRequest AlignReefRequest() {

        double distance = m_DistanceSensorSystem.LongestDistance();

        if (distance >= Constants.DistanceConstants.reefAlignedDistance)
            alignReefEdgeRequest.VelocityX = Constants.AutoAlignment.AutoReefAlignmentSpeed;
        else
            alignReefEdgeRequest.VelocityX = 0.0;

        return alignReefEdgeRequest;
    }

    private Pose2d RetrieveDestination(Constants.Alignment alignment) {
        // If nothing has been selected return nothing.

        // if (m_ElevatorDestination == Constants.Elevator.noDestination)
        // return null;
        if (m_selectedReef == Constants.AutoAlignment.noReef)
            return null;

        // Special case if the elevator is level one
        if ((alignment == Constants.Alignment.BRANCH) && (m_ElevatorDestination == Constants.Elevator.levelOne))
            alignment = Constants.Alignment.BRANCH;

        return vision[0].GetDestinationFromReefBranch(m_selectedReef, alignment);
    }

    private void SetReefBranch(int branch) {
        m_selectedReef = branch;
    }

    public boolean IsAlgeaHigh() {

        if ((m_selectedReef == 18) || (m_selectedReef == 20) || (m_selectedReef == 22) ||
                (m_selectedReef == 7) || (m_selectedReef == 11) || (m_selectedReef == 9))
            return true;
        else
            return false;
    }

    public AlgeaHeight GetAlgeaHeight() {
        // On Red odd april tags have high balls, on blue even april tags have high
        // balls
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.get() == Alliance.Red) {
            if (m_selectedReef % 2 != 0)
                return AlgeaHeight.Hi;
            else
                return AlgeaHeight.Low;

        } else if (ally.get() == Alliance.Blue) {
            if (m_selectedReef % 2 == 0)
                return AlgeaHeight.Hi;
            else
                return AlgeaHeight.Low;
        }
        return AlgeaHeight.Low;
    }

    private void AlignRobot(Constants.Alignment alignment) {
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d destinationPos = RetrieveDestination(alignment);

        if (destinationPos == null)
            return;

        // The rotation component in these poses represents the direction of travel
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, destinationPos);
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                new PathConstraints(
                        3, 3, // Velocity was 2.5
                        Units.degreesToRadians(360), Units.degreesToRadians(540)),
                null, // Ideal starting state can be null for on-the-fly paths
                new GoalEndState(0.0, destinationPos.getRotation())// currentPose.getRotation())
        );

        // Prevent this path from being flipped on the red alliance, since the given
        // positions are already correct
        path.preventFlipping = true;
        automaticPath = AutoBuilder.followPath(path).withTimeout(3); // Maybe longer timeout.
        automaticPath.schedule();
    }

    private void backUp(double distance) {
        Pose2d currentPose = drivetrain.getState().Pose;

        // try to calculate

        double x = currentPose.getX();
        double y = currentPose.getY();
        Rotation2d heading = currentPose.getRotation();
        double newX = x - distance * Math.cos(heading.getRadians());
        double newY = y - distance * Math.sin(heading.getRadians());

        Pose2d destinationPos = new Pose2d(newX, newY, heading);

        /*
         * Pose2d destinationPos = new Pose2d(
         * currentPose.getTranslation()
         * .plus(new Translation2d(0.1524, 0).rotateBy(currentPose.getRotation())),
         * currentPose.getRotation());
         */
        // Pose2d destinationPos = new Pose2d(14.8, 3.4,Rotation2d.fromDegrees(174.06));

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

        if (m_AutoAlignOff)
            return false;

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
                        () -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed * m_SlowSpeedMod) // Drive
                                                                                                            // forward
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // Y
                                                                                                            // (forward)
                                .withVelocityY(-driverController.getLeftX() * MaxSpeed * m_SlowSpeedMod) // Drive left
                                                                                                         // with
                                                                                                         // negative X
                                                                                                         // (left)
                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate * m_SlowSpeedMod)) // Drivecounterclockwise
                                                                                                                      // with
                                                                                                                      // negative
                                                                                                                      // X
                                                                                                                      // (left)
        );

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Braking
        new Trigger(() -> driverController.getRightTriggerAxis() > 0.25)
                .whileTrue(drivetrain.applyRequest(() -> brake));

        // Binding to call CancelAutomaticMovement() if any analog stick is over 50%
        new Trigger(() -> (Math.abs(driverController.getLeftX()) > 0.4) ||
                (Math.abs(driverController.getLeftY()) > 0.4) ||
                (Math.abs(driverController.getRightX()) > 0.4) ||
                (Math.abs(driverController.getRightY()) > 0.4) ||
                ManualOperator.getRawButtonPressed(Constants.ManualOperatorConstants.ABORT))
                .onTrue(new InstantCommand(() -> this.CancelAutomaticMovement())
                        .andThen(new InstantCommand(() -> SetScoreTrigger(false)))
                        .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))
                        .andThen(new InstantCommand(() -> m_Elevator.Stow()))
                        .andThen(new InstantCommand(() -> m_Effector.Stop())));

        new Trigger(() -> ManualOperator.getRawButton(Constants.ManualOperatorConstants.MANUAL_SWITCH))
                .onTrue(new InstantCommand(() -> EnableManualMode()))
                .onFalse(new InstantCommand(() -> DisableManualMode()));

        driverController.y().onTrue(
                drivetrain.applyRequest(() -> AlignReefRequest())
                        .withTimeout(2)
                        .until(() -> m_DistanceSensorSystem.CloseEnoughToReef())
                        .until(() -> DriverInterrupt()));

        driverController.b().onTrue(
                new WaitCommand(0.5)
                        // Alignment correction
                        .andThen(drivetrain.applyRequest(() -> AlignReefRequest())
                                .withTimeout(2)
                                .until(() -> m_DistanceSensorSystem.CloseEnoughToReef())
                                .until(() -> DriverInterrupt()))
                        .andThen(drivetrain.applyRequest(() -> AlignShotRequest())
                                .withTimeout(2)
                                .until(() -> m_DistanceSensorSystem.ReefScoreAligned())
                                .until(() -> DriverInterrupt()))
                        .andThen(drivetrain.applyRequest(() -> StopRobotNow()).withTimeout(0.1))

                        // Taken from scoring code
                        .andThen(new InstantCommand(() -> m_Elevator.LevelTwo()))
                        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                        .andThen(new InstantCommand(() -> m_Effector.ScoreCoral()))
                        .andThen(new WaitUntilCommand(() -> !m_Effector.Scoring()))
                        .andThen(new WaitCommand(0.15))
                        .andThen(new InstantCommand(() -> m_Elevator.Stow()))

                        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()).withTimeout(0.5))
                        .andThen(new WaitCommand(0.6))
                        .andThen(m_Elevator.RunCurrentZeroing()) // Todo make a proper reverse.

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

        new JoystickButton(ManualOperator, Constants.ManualOperatorConstants.ARM_CLIMBER)
                .onTrue(new InstantCommand(() -> ArmClimber()));

        // Winch Code
        new JoystickButton(ManualOperator, Constants.ManualOperatorConstants.CLIMBER_DN)
                .onTrue(new InstantCommand(() -> m_Elevator.SetLevel(Constants.Elevator.levelTwo)))
                .whileTrue(new ClimberDownCommand(m_Climber));
        new JoystickButton(ManualOperator, Constants.ManualOperatorConstants.CLIMBER_UP)
                .onTrue(new InstantCommand(() -> m_Elevator.SetLevel(Constants.Elevator.levelTwo)))
                .whileTrue(new ClimberUpCommand(m_Climber));

        // Code to make buttons set elevator destination
        JoystickButton l1Button = new JoystickButton(ElevatorOperator, Constants.ElevatorOperatorConstants.L1);
        new Trigger(() -> l1Button.getAsBoolean() && !ManualModeEngaged())
                .onTrue(new InstantCommand(() -> SetElevatorDestination(Constants.Elevator.levelOne)))
                .onTrue(new InstantCommand(() -> m_L1Scoring = true)
                        .andThen(new InstantCommand(() -> disableAlgeaButtonPressed())));

        new Trigger(() -> l1Button.getAsBoolean() && ManualModeEngaged())
                .onTrue(new InstantCommand(() -> m_Elevator.SetLevel(Constants.Elevator.levelOne)));

        JoystickButton l2Button = new JoystickButton(ElevatorOperator, Constants.ElevatorOperatorConstants.L2);
        new Trigger(() -> l2Button.getAsBoolean() && !ManualModeEngaged())
                .onTrue(new InstantCommand(() -> SetElevatorDestination(Constants.Elevator.levelTwo))
                        .andThen(new InstantCommand(() -> disableAlgeaButtonPressed()))
                        .andThen(new InstantCommand(() -> m_L1Scoring = false)));

        new Trigger(() -> l2Button.getAsBoolean() && ManualModeEngaged())
                .onTrue(new InstantCommand(() -> m_Elevator.SetLevel(Constants.Elevator.levelTwo)));

        JoystickButton l3Button = new JoystickButton(ElevatorOperator, Constants.ElevatorOperatorConstants.L3);
        new Trigger(() -> l3Button.getAsBoolean() && !ManualModeEngaged())
                .onTrue(new InstantCommand(() -> SetElevatorDestination(Constants.Elevator.levelThree))
                        .andThen(new InstantCommand(() -> disableAlgeaButtonPressed()))
                        .andThen(new InstantCommand(() -> m_L1Scoring = false)));

        new Trigger(() -> l3Button.getAsBoolean() && ManualModeEngaged())
                .onTrue(new InstantCommand(() -> m_Elevator.SetLevel(Constants.Elevator.levelThree)));

        JoystickButton l4Button = new JoystickButton(ElevatorOperator, Constants.ElevatorOperatorConstants.L4);
        new Trigger(() -> l4Button.getAsBoolean() && !ManualModeEngaged())
                .onTrue(new InstantCommand(() -> SetElevatorDestination(Constants.Elevator.levelFour))
                        .andThen(new InstantCommand(() -> disableAlgeaButtonPressed()))
                        .andThen(new InstantCommand(() -> m_L1Scoring = false)));

        new Trigger(() -> l4Button.getAsBoolean() && ManualModeEngaged())
                .onTrue(new InstantCommand(() -> m_Elevator.SetLevel(Constants.Elevator.levelFour)));

        // Bindings to control Elevator Level, wait until it aligns, then runs
        // sequential command gorup to score

        JoystickButton twelveLeftButton = new JoystickButton(ReefOperator, Constants.ReefOperatorConstants.TWELVE_LEFT);
        new Trigger(() -> twelveLeftButton.getAsBoolean() && ElevatorHasDestination())
                .onTrue(new InstantCommand(() -> NewScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.TWELVE_LEFT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.BRANCH)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetScoreTrigger(true))));

        JoystickButton twelveRightButton = new JoystickButton(ReefOperator,
                Constants.ReefOperatorConstants.TWELVE_RIGHT);
        new Trigger(() -> twelveRightButton.getAsBoolean() && ElevatorHasDestination())
                .onTrue(new InstantCommand(() -> NewScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.TWELVE_RIGHT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.BRANCH)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetScoreTrigger(true))));

        JoystickButton tenLeftButton = new JoystickButton(ReefOperator, Constants.ReefOperatorConstants.TEN_LEFT);
        new Trigger(() -> tenLeftButton.getAsBoolean() && ElevatorHasDestination())

                .onTrue(new InstantCommand(() -> NewScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.TEN_LEFT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.BRANCH)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetScoreTrigger(true))));

        JoystickButton tenRightButton = new JoystickButton(ReefOperator, Constants.ReefOperatorConstants.TEN_RIGHT);
        new Trigger(() -> tenRightButton.getAsBoolean() && ElevatorHasDestination())
                .onTrue(new InstantCommand(() -> NewScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.TEN_RIGHT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.BRANCH)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetScoreTrigger(true))));

        JoystickButton eightLeftButton = new JoystickButton(ReefOperator, Constants.ReefOperatorConstants.EIGHT_LEFT);
        new Trigger(() -> eightLeftButton.getAsBoolean() && ElevatorHasDestination())
                .onTrue(new InstantCommand(() -> NewScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.EIGHT_LEFT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.BRANCH)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetScoreTrigger(true))));

        JoystickButton eightRightButton = new JoystickButton(ReefOperator, Constants.ReefOperatorConstants.EIGHT_RIGHT);
        new Trigger(() -> eightRightButton.getAsBoolean() && ElevatorHasDestination())
                .onTrue(new InstantCommand(() -> NewScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.EIGHT_RIGHT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.BRANCH)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetScoreTrigger(true))));

        JoystickButton sixLeftButton = new JoystickButton(ReefOperator, Constants.ReefOperatorConstants.SIX_LEFT);
        new Trigger(() -> sixLeftButton.getAsBoolean() && ElevatorHasDestination())
                .onTrue(new InstantCommand(() -> NewScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.SIX_LEFT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.BRANCH)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetScoreTrigger(true))));

        JoystickButton sixRightButton = new JoystickButton(ReefOperator, Constants.ReefOperatorConstants.SIX_RIGHT);
        new Trigger(() -> sixRightButton.getAsBoolean() && ElevatorHasDestination())
                .onTrue(new InstantCommand(() -> NewScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.SIX_RIGHT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.BRANCH)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetScoreTrigger(true))));

        JoystickButton fourLeftButton = new JoystickButton(ReefOperator, Constants.ReefOperatorConstants.FOUR_LEFT);
        new Trigger(() -> fourLeftButton.getAsBoolean() && ElevatorHasDestination())
                .onTrue(new InstantCommand(() -> NewScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.FOUR_LEFT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.BRANCH)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetScoreTrigger(true))));

        JoystickButton fourRightButton = new JoystickButton(ReefOperator, Constants.ReefOperatorConstants.FOUR_RIGHT);
        new Trigger(() -> fourRightButton.getAsBoolean() && ElevatorHasDestination())
                .onTrue(new InstantCommand(() -> NewScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.FOUR_RIGHT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.BRANCH)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetScoreTrigger(true))));

        JoystickButton twoLeftButton = new JoystickButton(ReefOperator, Constants.ReefOperatorConstants.TWO_LEFT);
        new Trigger(() -> twoLeftButton.getAsBoolean() && ElevatorHasDestination())
                .onTrue(new InstantCommand(() -> NewScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.TWO_LEFT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.BRANCH)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetScoreTrigger(true))));

        JoystickButton twoRightButton = new JoystickButton(ReefOperator, Constants.ReefOperatorConstants.TWO_RIGHT);
        new Trigger(() -> twoRightButton.getAsBoolean() && ElevatorHasDestination())
                .onTrue(new InstantCommand(() -> NewScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.TWO_RIGHT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.BRANCH)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetScoreTrigger(true))));

        // Complete Scoring Action trigger
        new Trigger(() -> GetScoreTrigger()).onTrue(
                new InstantCommand(() -> SetScoreTrigger(false))
                        .andThen(new InstantCommand(() -> m_Elevator.ResetEncoders()))
                        .andThen(new InstantCommand(() -> System.out.println("Scoring Trigger Activated.\n")))

                        // Add the magic wait here
                        .andThen(new WaitCommand(0.15)
                                // Align - Get close to reef (Tre look here)
                                .andThen(drivetrain.applyRequest(() -> AlignReefRequest())
                                        .withTimeout(0.33)
                                        .until(() -> m_DistanceSensorSystem.CloseEnoughToReef())
                                        .until(() -> DriverInterrupt()))

                                // Align to target (Tre look here)
                                .andThen(drivetrain.applyRequest(() -> AlignShotRequest())
                                        .withTimeout(1.5)
                                        .until(() -> m_DistanceSensorSystem.ReefScoreAligned())
                                        .until(() -> DriverInterrupt()))

                                .andThen(drivetrain.applyRequest(() -> StopRobotNow()) // Make sure the drive shuts off
                                        .withTimeout(0.1))

                                // Score
                                .andThen(new InstantCommand(() -> m_Elevator.SetLevel(GetElevatorDestination())))
                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                                .andThen(
                                        new InstantCommand(() -> m_Effector.ScoreCoralTeleOp(GetElevatorDestination())))
                                .andThen(new WaitUntilCommand(() -> !m_Effector.Scoring()))
                                .andThen(new WaitCommand(0.15))
                                .andThen(new InstantCommand(() -> m_Elevator.Stow()))

                                // Reset lift
                                .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()).withTimeout(2))
                                .andThen(new WaitCommand(0.6)) // was 0.6
                                .andThen(m_Elevator.RunCurrentZeroing())
                                .andThen(new InstantCommand(() -> resetElevatorDestination()))

                        // Chain command here instead of resetting.
                        // .andThen(new InstantCommand(() -> scoreTriggerActive = true)) // Set the flag
                        // to false

                        ));

        new JoystickButton(ElevatorOperator, Constants.ElevatorOperatorConstants.RESET)
                .onTrue(new ParallelCommandGroup(
                        m_Elevator.RunCurrentZeroing(),
                        new InstantCommand(() -> m_Effector.Stop())));

        new JoystickButton(ManualOperator, Constants.ManualOperatorConstants.MANUAL_SCORE)
                .onTrue(new InstantCommand(() -> m_Effector.ScoreCoral()));

        JoystickButton algeaButton = new JoystickButton(ElevatorOperator, Constants.ElevatorOperatorConstants.ALGAE);
        new Trigger(() -> algeaButton.getAsBoolean())
                .onTrue(new InstantCommand(() -> this.recordAlgeaButtonPressed())
                        .andThen(new InstantCommand(() -> SetElevatorDestination(Constants.Elevator.noDestination))));

        // Algea score trigger
        new Trigger(
                () -> (twelveLeftButton.getAsBoolean() || twelveRightButton.getAsBoolean()) && getAlgeaButtonPressed())
                .onTrue(new InstantCommand(() -> NewAlgeaScoreAttempt())
                        .andThen(new InstantCommand(() -> this.disableAlgeaButtonPressed()))
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.TWELVE_LEFT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.ALGEA)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetAlgeaScoreFlag(true))));

        new Trigger(
                () -> (tenLeftButton.getAsBoolean() || tenRightButton.getAsBoolean()) && getAlgeaButtonPressed())
                .onTrue(new InstantCommand(() -> NewAlgeaScoreAttempt())
                        .andThen(new InstantCommand(() -> this.disableAlgeaButtonPressed()))
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.TEN_LEFT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.ALGEA)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetAlgeaScoreFlag(true))));

        new Trigger(
                () -> (eightLeftButton.getAsBoolean() || eightRightButton.getAsBoolean()) && getAlgeaButtonPressed())
                .onTrue(new InstantCommand(() -> NewAlgeaScoreAttempt())
                        .andThen(new InstantCommand(() -> this.disableAlgeaButtonPressed()))
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.EIGHT_LEFT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.ALGEA)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetAlgeaScoreFlag(true))));

        new Trigger(
                () -> (sixLeftButton.getAsBoolean() || sixRightButton.getAsBoolean()) && getAlgeaButtonPressed())
                .onTrue(new InstantCommand(() -> NewAlgeaScoreAttempt())
                        .andThen(new InstantCommand(() -> this.disableAlgeaButtonPressed()))
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.SIX_LEFT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.ALGEA)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetAlgeaScoreFlag(true))));

        new Trigger(
                () -> (fourLeftButton.getAsBoolean() || fourRightButton.getAsBoolean()) && getAlgeaButtonPressed())
                .onTrue(new InstantCommand(() -> NewAlgeaScoreAttempt())
                        .andThen(new InstantCommand(() -> this.disableAlgeaButtonPressed()))
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.FOUR_LEFT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.ALGEA)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetAlgeaScoreFlag(true))));

        new Trigger(
                () -> (twoLeftButton.getAsBoolean() || twoRightButton.getAsBoolean()) && getAlgeaButtonPressed())
                .onTrue(new InstantCommand(() -> NewAlgeaScoreAttempt())
                        .andThen(new InstantCommand(() -> SetReefBranch(Constants.ReefOperatorConstants.TWO_LEFT)))
                        .andThen(new InstantCommand(() -> this.AlignRobot(Constants.Alignment.ALGEA)))
                        .andThen(new WaitUntilCommand(() -> !autoPathActive()))
                        .andThen(new InstantCommand(() -> SetAlgeaScoreFlag(true))));

        // New trigger for the algea score
        new Trigger(() -> GetAlgeaScoreTrigger() && IsAlgeaHigh()).onTrue(
                new InstantCommand(() -> SetAlgeaScoreFlag(false))
                        // Add the magic wait here
                        .andThen(new WaitCommand(0.15))
                        .andThen(new InstantCommand(() -> resetReefBranch()))
                        .andThen(new InstantCommand(() -> m_Elevator.AlgeaHigh()))
                        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                        .andThen(new InstantCommand(() -> m_Effector.MoveAlgeaArm())
                                .andThen(new WaitCommand(0.5))
                                .andThen(new InstantCommand(() -> m_Effector.Stop()))
                                .andThen(new InstantCommand(() -> this.backUp(0.5)))
                                .andThen(new WaitUntilCommand(() -> m_Effector.reachedSetState()))
                                .andThen(new InstantCommand(() -> this.backUp(0.25)))
                                .andThen(new InstantCommand(() -> m_Elevator.Stow()))));
        // Clear the low one
        new Trigger(() -> GetAlgeaScoreTrigger() && !IsAlgeaHigh()).onTrue(
                new InstantCommand(() -> SetScoreTrigger(false))
                        // Add the magic wait here
                        .andThen(new WaitCommand(0.15))
                        .andThen(new InstantCommand(() -> resetReefBranch()))
                        .andThen(new InstantCommand(() -> m_Elevator.AlgaeCheckpoint()))
                        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                        .andThen(new InstantCommand(() -> m_Effector.MoveAlgeaArm()))
                        .andThen(new InstantCommand(() -> m_Elevator.AlgeaLow()))
                        .andThen(new WaitUntilCommand(() -> m_Elevator.reachedSetState()))
                        .andThen(new WaitCommand(0.15))
                        .andThen(new InstantCommand(() -> m_Effector.Stop()))
                        .andThen(new InstantCommand(() -> this.backUp(0.5)))
                        .andThen(new WaitUntilCommand(() -> m_Effector.reachedSetState()))
                        .andThen(new InstantCommand(() -> m_Elevator.Stow())));

        // Turn on coral intake with left button press
        new JoystickButton(ElevatorOperator, Constants.ElevatorOperatorConstants.INTAKE)
                .onTrue(new InstantCommand(() -> m_Effector.IntakeCoral()));

        new JoystickButton(ElevatorOperator, Constants.ElevatorOperatorConstants.REV)
                .whileTrue(new IntakeReverse(m_Effector));

        // drivetrain.registerTelemetry(logger::telemeterize);
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
