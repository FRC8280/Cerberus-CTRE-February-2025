package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DistanceSensorSystem;

public class AlignRobotReef extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final DistanceSensorSystem m_distanceSystem;
    private final CommandXboxController m_driverController;
    SwerveRequest.RobotCentric m_robotCentricDrive;
    private final PIDController pidController;
    private Timer m_EmergencyTimer;


    public AlignRobotReef(SwerveRequest.RobotCentric robotCentricDrive, CommandSwerveDrivetrain drivetrain, DistanceSensorSystem distanceSystem, CommandXboxController driverController) {
        this.m_drivetrain = drivetrain;
        this.m_distanceSystem = distanceSystem;
        this.m_driverController = driverController;
        this.m_robotCentricDrive = robotCentricDrive;
        this.pidController = new PIDController(0.1, 0.0, 0.0); // Adjust PID constants as needed
        //addRequirements(drivetrain, distanceSystem);
        m_EmergencyTimer = new Timer();
    }
        
    @Override
    public void initialize() {
        // Initialization code if needed
        pidController.reset();
        m_EmergencyTimer.reset();
        m_EmergencyTimer.start();
        
    }

    @Override
    public void execute() {
        // Read the distance values from the forward-facing CANRange sensors
        double frontDistance1 = m_distanceSystem.m_alignRanges[0].getDistance().refresh().getValueAsDouble() ;
        double frontDistance2 = m_distanceSystem.m_alignRanges[1].getDistance().refresh().getValueAsDouble() ;

        // Calculate the average distance
        double averageDistance = (frontDistance1 + frontDistance2) / 2.0;

        // Calculate the PID output
        double output = capOutput(pidController.calculate(averageDistance, Constants.DistanceConstants.reefAlignedDistance)); // Target distance is 0.4
        
        m_drivetrain.applyRequest( () -> m_robotCentricDrive.withVelocityX(0.25).withVelocityY(0));
        // Move the robot forward using the PID output
        /*m_drivetrain.applyRequest(
            () -> m_robotCentricDrive.withVelocityX(output) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0) // Drivecounterclockwise with negative X (left)
            ); */

            
    }

    private double capOutput(double output) {
        return Math.max(-Constants.DistanceConstants.maxSpeed, Math.min(Constants.DistanceConstants.maxSpeed, output));
    }

    public boolean aboveThreshold(double value, double constant) {
        return value >= Math.abs(0.95 * constant);
    }

    @Override
    public boolean isFinished() {
        // Check if both distance sensors read zero
        double frontDistance1 = m_distanceSystem.m_alignRanges[0].getDistance().refresh().getValueAsDouble();
        double frontDistance2 = m_distanceSystem.m_alignRanges[1].getDistance().refresh().getValueAsDouble() ;

        return false;

        /*if(m_EmergencyTimer.hasElapsed(Constants.DistanceConstants.maxTimer))
            return true;

        if(isInterrupted())
            return true;

        boolean leftthreshold = aboveThreshold(frontDistance1,Constants.DistanceConstants.reefScoringDistance);
        boolean rightthreshold = aboveThreshold(frontDistance2,Constants.DistanceConstants.reefScoringDistance);
        
        if(frontDistance1 > Constants.DistanceConstants.reefScoringDistance && frontDistance2 > Constants.DistanceConstants.reefScoringDistance)
            return false;
        else 
            return true;*/
    }

    @Override
    public void end(boolean interrupted) {

        int x = 4;
        x = 1+x;

        // Stop the robot
       /* m_drivetrain.applyRequest(
            () -> m_robotCentricDrive.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0) // Drivecounterclockwise with negative X (left)
            );  */
    }

    public boolean isInterrupted() {
        // Abort if any joystick is moved past 50%
        return Math.abs(m_driverController.getLeftX()) > 0.5 ||
               Math.abs(m_driverController.getLeftY()) > 0.5 ||
               Math.abs(m_driverController.getRightX()) > 0.5;
    }
}