// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Effector;
import edu.wpi.first.wpilibj2.command.Command;

/** An ClimberUpCommand that uses a climb subsystem. */
public class IntakeReverse extends Command {
  private final Effector m_Effector;

  /**
   * Runs the climber up, note that this can change 
   * based on how the winch is wound.
   *
   * @param climber The subsystem used by this command.
   */
  public IntakeReverse(Effector effector) {
    m_Effector = effector;
    addRequirements(effector);
      }
    
      // Called when the command is initially scheduled.
  @Override
    public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Effector.EjectCoral();
  }

  // Called once the command ends or is interrupted.. Here we ensure the climber is not
  // running once we let go of the button
  @Override
  public void end(boolean interrupted) {
    m_Effector.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}