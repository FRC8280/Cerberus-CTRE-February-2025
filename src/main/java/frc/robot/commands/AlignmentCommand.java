// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignmentCommand extends Command {
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*

    Check is this max or min extension? Which direction is out. 
     * if(!m_climber.AtMaxExtensions()) 
        m_climber.runClimber(ClimberConstants.CLIMBER_SPEED_UP);
    else
        m_climber.runClimber(0);
     */
  }

  // Called once the command ends or is interrupted. Here we ensure the climber is not
  // running once we let go of the button
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //if(!m_climber.AtMaxExtensions())
    //return false?
    return false;
  }
}