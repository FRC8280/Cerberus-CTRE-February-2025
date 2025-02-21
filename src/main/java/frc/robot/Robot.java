// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonUtils;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static int aprilTagId;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;
  //private Vision vision;
   private final Field2d m_field = new Field2d();

  public Robot() {
    CanBridge.runTCP();
    //vision = new Vision();
    m_robotContainer = new RobotContainer();
    SmartDashboard.putData("Field", m_field);
    
    
  }

 /* public void processAprilTags() {
    PhotonPipelineResult result = m_robotContainer.vision.camera.getLatestResult();
    if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
            int aprilTagId = target.getFiducialId();
            m_robotContainer.vision.CalculateAutoReefTarget(aprilTagId);
            SmartDashboard.putNumber("Detected April Tag", aprilTagId);
            //System.out.println("Detected AprilTag ID: " + aprilTagId);
        }
    } else {
        //System.out.println("No targets detected.");
    }
}*/

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Correct pose estimate with vision measurements
    /*var visionEst = m_robotContainer.vision.getEstimatedGlobalPose();
    visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = m_robotContainer.vision.getEstimationStdDevs();

                m_robotContainer.drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
    processAprilTags();    
    m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
    SmartDashboard.putNumber("Current Drive X", m_robotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("Current Drive Y", m_robotContainer.drivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("Current Yaw", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());*/
    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *m_robotContainer.drivetrain.getState().Pose
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
