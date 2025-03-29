// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import org.photonvision.PhotonUtils;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public boolean debugSpew = false;
  public static int aprilTagId;

  private final RobotContainer m_robotContainer;
  // private Vision vision;
  private final Field2d m_field = new Field2d();

  public Robot() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void robotInit() {
    PortForwarder.add(5800, "photonvision-1.local", 5800);
    PortForwarder.add(5800, "photonvision-2.local", 5800);
    PortForwarder.add(5800, "photonvision-3.local", 5800);
    SignalLogger.enableAutoLogging(false);
    m_robotContainer.threadCommand();
  }

  @Override
  public void robotPeriodic() {
    /*
     * if(m_robotContainer!=null)
     * m_robotContainer.Periodic();
     */

    CommandScheduler.getInstance().run();

    // Correct pose estimate with vision measurements
   
   for (int i = 0; i < m_robotContainer.vision.length; i++) {
      var visionEst = m_robotContainer.vision[i].getEstimatedGlobalPose();
      int index = i;
      visionEst.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = m_robotContainer.vision[index].getEstimationStdDevs();

            m_robotContainer.drivetrain.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });
    }  
    
    if(debugSpew){
        m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
        SmartDashboard.putNumber("Current Drive X",
     m_robotContainer.drivetrain.getState().Pose.getX());
     SmartDashboard.putNumber("Current Drive Y",
     m_robotContainer.drivetrain.getState().Pose.getY());
     SmartDashboard.putNumber("Current Yaw",
     m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
    }
     
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    // Clear the CommandScheduler of all commands
    CommandScheduler.getInstance().cancelAll();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Clear the CommandScheduler of all commands
    //CommandScheduler.getInstance().cancelAll();
    m_robotContainer.RobotInit();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
