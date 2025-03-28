package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DistanceSensorSystem;

public class AlignRobotTarget extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final DistanceSensorSystem m_distanceSystem;
    private final CommandXboxController m_driverController;
    SwerveRequest.RobotCentric m_robotCentricDrive;
    private final PIDController pidController;


    public AlignRobotTarget(SwerveRequest.RobotCentric robotCentricDrive, CommandSwerveDrivetrain drivetrain, DistanceSensorSystem distanceSystem, CommandXboxController driverController) {
        this.m_drivetrain = drivetrain;
        this.m_distanceSystem = distanceSystem;
        this.m_driverController = driverController;
        this.m_robotCentricDrive = robotCentricDrive;
        this.pidController = new PIDController(0.1, 0.0, 0.0); // Adjust PID constants as needed
        addRequirements(drivetrain, distanceSystem);
    }
        
    @Override
    public void initialize() {
        // Initialization code if needed
        pidController.reset();
    }

    @Override
    public void execute() {
        // Read the distance values from the forward-facing CANRange sensors
        double targetDistance0 = m_distanceSystem.m_TargetArray[0].getDistance().refresh().getValueAsDouble() ;
        double targetDistance1 = m_distanceSystem.m_TargetArray[1].getDistance().refresh().getValueAsDouble() ;
        double targetDistance2 = m_distanceSystem.m_TargetArray[2].getDistance().refresh().getValueAsDouble() ;
        double targetDistance3 = m_distanceSystem.m_TargetArray[3].getDistance().refresh().getValueAsDouble() ;
        double targetDistance4 = m_distanceSystem.m_TargetArray[4].getDistance().refresh().getValueAsDouble() ;
        // Calculate the average distance
        double averageDistance = (targetDistance0 + targetDistance1 + targetDistance2
         + targetDistance3 + targetDistance4) / 5.0;

        // Calculate the PID output
        double output = pidController.calculate(averageDistance, Constants.DistanceConstants.reefScoringDistance); // Target distance is 0.4

        // Move the robot forward using the PID output
        m_drivetrain.applyRequest(
            () -> m_robotCentricDrive.withVelocityX(output) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0) // Drivecounterclockwise with negative X (left)
            ); 
    }

    public boolean aboveThreshold(double value, double constant) {
        return value >= Math.abs(0.95 * constant);
    }

    @Override
    public boolean isFinished() {
        // Check if both distance sensors read zero
        double targetDistance0 = m_distanceSystem.m_TargetArray[0].getDistance().refresh().getValueAsDouble();
        double targetDistance1 = m_distanceSystem.m_TargetArray[1].getDistance().refresh().getValueAsDouble();
        double targetDistance2 = m_distanceSystem.m_TargetArray[2].getDistance().refresh().getValueAsDouble();
        double targetDistance3 = m_distanceSystem.m_TargetArray[3].getDistance().refresh().getValueAsDouble();
        double targetDistance4 = m_distanceSystem.m_TargetArray[4].getDistance().refresh().getValueAsDouble();

        if(isInterrupted())
            return true;

        return aboveThreshold(targetDistance0,Constants.DistanceConstants.reefAlignedDistance) && 
                aboveThreshold(targetDistance1,Constants.DistanceConstants.reefAlignedDistance) && 
                aboveThreshold(targetDistance2,Constants.DistanceConstants.reefAlignedDistance) && 
                aboveThreshold(targetDistance3,Constants.DistanceConstants.reefAlignedDistance) && 
                aboveThreshold(targetDistance4,Constants.DistanceConstants.reefAlignedDistance);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        m_drivetrain.applyRequest(
            () -> m_robotCentricDrive.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0) // Drivecounterclockwise with negative X (left)
            ); 
    }

    public boolean isInterrupted() {
        // Abort if any joystick is moved past 50%
        return Math.abs(m_driverController.getLeftX()) > 0.5 ||
               Math.abs(m_driverController.getLeftY()) > 0.5 ||
               Math.abs(m_driverController.getRightX()) > 0.5;
    }
}