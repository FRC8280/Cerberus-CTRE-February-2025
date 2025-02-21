package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveToPositionCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final double targetX;
    private final double targetY;
    private final double targetYaw;

    public MoveToPositionCommand(CommandSwerveDrivetrain drivetrain, double targetX, double targetY, double yaw) {
        this.drivetrain = drivetrain;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetYaw = yaw;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Initialize the command, if needed
    }

    @Override
    public void execute() {
        // Move the robot to the target position
        //drivetrain.moveToPosition(targetX, targetY,targetYaw);
        
    }

    @Override
    public boolean isFinished() {
        // Check if the robot has reached the target position
        //return drivetrain.isAtPosition(targetX, targetY,targetYaw);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        //drivetrain.stop();
    }
}