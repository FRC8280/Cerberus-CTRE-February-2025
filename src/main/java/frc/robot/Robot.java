// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.targeting.PhotonTrackedTarget;

import au.grapplerobotics.CanBridge;
/*import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;*/
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.PhotonUtils;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static int aprilTagId;

  private final RobotContainer m_robotContainer;
  // private Vision vision;
  private final Field2d m_field = new Field2d();

  public Robot() {
    CanBridge.runTCP();
    m_robotContainer = new RobotContainer();
    SmartDashboard.putData("Field", m_field);
  }

  public void processAprilTags() {
    PhotonPipelineResult camera1Result = m_robotContainer.vision[0].camera.getLatestResult();
    PhotonPipelineResult camera2Result = m_robotContainer.vision[1].camera.getLatestResult();

    double LowestDistance = 9999.0;
    double distance = 0;

    for (int i = 0; i < 2; i++) {
      if (camera1Result.hasTargets() || camera2Result.hasTargets()) {
        PhotonPipelineResult result = camera1Result.hasTargets() ? camera1Result : camera2Result;
        for (PhotonTrackedTarget target : result.getTargets()) {
          int aprilTagId = target.getFiducialId();
          if (m_robotContainer.vision[0].CheckValidAprilTag(aprilTagId)) {

               distance = m_robotContainer.vision[0].calculateDistanceBetweenPoseAndTransform(
                m_robotContainer.drivetrain.getState().Pose,target.getBestCameraToTarget());
                //System.out.println("AprilTag ID: " + aprilTagId +" At Distance: " + distance);

            if (distance < LowestDistance) {
              m_robotContainer.m_ReefTargets = m_robotContainer.vision[0].CalculateAutoReefTarget(aprilTagId);
              SmartDashboard.putNumber("Detected April Tag", aprilTagId);
              
              if(m_robotContainer.m_ReefTargets.leftTarget!=null)
                LowestDistance = distance;
            }
          }
        }
      } else {
        // System.out.println("No targets detected.");
      }
    }
  }

  @Override
  public void robotInit() {
    // Clear the CommandScheduler of all commands
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.RobotInit();
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

    processAprilTags();
    m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);

    
     SmartDashboard.putNumber("Current Drive X",
     m_robotContainer.drivetrain.getState().Pose.getX());
     SmartDashboard.putNumber("Current Drive Y",
     m_robotContainer.drivetrain.getState().Pose.getY());
     SmartDashboard.putNumber("Current Yaw",
     m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
     /* 
     * if(m_robotContainer.m_ReefTargets.leftTarget!=null){
     * SmartDashboard.putNumber("Target Left X",
     * m_robotContainer.m_ReefTargets.leftTarget.getX());
     * SmartDashboard.putNumber("Target Left Y",
     * m_robotContainer.m_ReefTargets.leftTarget.getY());
     * SmartDashboard.putNumber("Target Left Yaw",
     * m_robotContainer.m_ReefTargets.leftTarget.getRotation().getDegrees());
     * }
     * 
     * if(m_robotContainer.m_ReefTargets.rightTarget!=null){
     * SmartDashboard.putNumber("Target Right X",
     * m_robotContainer.m_ReefTargets.rightTarget.getX());
     * SmartDashboard.putNumber("Target Right Y",
     * m_robotContainer.m_ReefTargets.rightTarget.getY());
     * SmartDashboard.putNumber("Target Right Yaw",
     * m_robotContainer.m_ReefTargets.rightTarget.getRotation().getDegrees());
     * }
     */
    // Todo add smart dashbaord for vision targets .
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
