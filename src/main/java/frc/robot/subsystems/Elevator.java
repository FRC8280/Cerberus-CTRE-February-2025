package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.hardware.TalonFX;
import au.grapplerobotics.LaserCan;
import frc.robot.Constants;
//import frc.robot.subsystems.RobotMap.mapElevator;


public class Elevator extends SubsystemBase {

  private TalonFX m_FrontMotor;
  private TalonFX m_RearMotor;

  //Control for elevator

  public enum ElevatorState {
    L4,
    L3,
    L2,
    L1,
    STOWED
  }

  //private ElevatorState elevatorState = ElevatorState.STOWED;
  private double targetElevatorPosition = Constants.Elevator.kStowed;

  private static final Slot0Configs elevatorGains = new Slot0Configs();
  
  /** Creates a new ShooterSubsystem. */
  public Elevator() {

    //*/ create new TalonFXs and configure them
    m_FrontMotor = new TalonFX(Constants.Elevator.kFrontCanId);
    m_RearMotor = new TalonFX(Constants.Elevator.kRearCanId);

    TalonFXConfiguration ReartalonFXConfigs = new TalonFXConfiguration();
    ReartalonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_RearMotor.getConfigurator().apply(ReartalonFXConfigs);

    
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 40;
    m_FrontMotor.getConfigurator().apply(currentLimits);
    m_RearMotor.getConfigurator().apply(currentLimits);
    
    elevatorGains.kP = 2.5;
    elevatorGains.kI = 0.6;//0.0;
    elevatorGains.kD = 0.25;
    elevatorGains.kG = 0.09;
    //elevatorGains.kV = 7.52;
    //elevatorGains.kA = 0.01;
    m_RearMotor.getConfigurator().apply(elevatorGains);
    ResetEncoders();
  }

  public boolean ElevatorAtZero()
  {
    return m_RearMotor.getPosition().getValueAsDouble() > -1 && m_FrontMotor.getPosition().getValueAsDouble() <1;
  }

  //manual pivot commands to test launcher
  //Function to set up position control of motors
  public void setPosition(double setpoint) {
    final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    m_RearMotor.setControl(positionRequest.withPosition(setpoint));

    //Set the front motor to follow the rear motor
    m_FrontMotor.setControl(new Follower(m_RearMotor.getDeviceID(), true));
  }

  //Functions to set the elevator to different positions
  public void Stow(){
    setPosition(Constants.Elevator.kStowed);
    targetElevatorPosition = Constants.Elevator.kStowed;
  }

  public void UpLevel()
  {
      if(targetElevatorPosition < Constants.Elevator.levelOne)
      {
        SetLevel(1);
        return;
      }

      else if((targetElevatorPosition >= Constants.Elevator.levelOne) && (targetElevatorPosition < Constants.Elevator.levelTwo))
      {
        SetLevel(2);
        return;
      }

      else if((targetElevatorPosition == Constants.Elevator.levelTwo) && (targetElevatorPosition < Constants.Elevator.levelThree))
      {
        SetLevel(3);
        return;
      }

      else if((targetElevatorPosition == Constants.Elevator.levelThree) && (targetElevatorPosition < Constants.Elevator.levelFour))
      {
        SetLevel(4);
        return;
      }
      
      return;
  }

  public void DownLevel()
  {
      if(targetElevatorPosition <= Constants.Elevator.levelOne)
      {
        Stow();
        return;
      }

      if( (targetElevatorPosition > Constants.Elevator.levelOne) && (targetElevatorPosition <= Constants.Elevator.levelTwo))
      {
        SetLevel(1);
        return;
      }

      if((targetElevatorPosition > Constants.Elevator.levelTwo) && (targetElevatorPosition <= Constants.Elevator.levelThree))
      {
        SetLevel(2);
        return;
      }

      if((targetElevatorPosition > Constants.Elevator.levelThree) && (targetElevatorPosition <= Constants.Elevator.levelFour))
      {
        SetLevel(3);
        return;
      }
      
      return;
  }


  public void SetLevel(int level){

    switch(level){
      case 1:
        setPosition(Constants.Elevator.levelOne);
        targetElevatorPosition = Constants.Elevator.levelOne;
        break;
      case 2:
        setPosition(Constants.Elevator.levelTwo);
        targetElevatorPosition = Constants.Elevator.levelTwo;
        break;
      case 3:
        setPosition(Constants.Elevator.levelThree);
        targetElevatorPosition = Constants.Elevator.levelThree;
        break;
      case 4:
        setPosition(Constants.Elevator.levelFour);
        targetElevatorPosition = Constants.Elevator.levelFour;
        break;
    }
  }

  public void AlgeaHigh()
  {
    setPosition(19.07);
    targetElevatorPosition = 19.07;
  }

  public void AlgeaLow()
  {
    setPosition(9.036);
    targetElevatorPosition = 9.036;
  }

  public void AlgaeCheckpoint(){
    setPosition(2.279);
    targetElevatorPosition = 2.279;
  }

  public void LevelOne()
  {
    setPosition(Constants.Elevator.levelOne);
    targetElevatorPosition = Constants.Elevator.levelOne;
  }

  public void LevelTwo()
  {
    setPosition(Constants.Elevator.levelTwo);
    targetElevatorPosition = Constants.Elevator.levelTwo;
  }

  public void LevelThree()
  {
     setPosition(Constants.Elevator.levelThree);
     targetElevatorPosition = Constants.Elevator.levelThree;
  }

  public void LevelFour()
  {
     setPosition(Constants.Elevator.levelFour);
     targetElevatorPosition = Constants.Elevator.levelFour;
  }

  public void LevelFourReset(){
    m_RearMotor.setPosition(Constants.Elevator.levelFour);
  }

  //Functions to check if the elevator has reached the set position
  public boolean aboveThreshold(double value, double constant) {
    return value >= Math.abs(0.95 * constant);
}

public boolean reachedSetState() {
  double currentPosition = m_RearMotor.getPosition().getValueAsDouble();
  return aboveThreshold(currentPosition, targetElevatorPosition);
}

public boolean atStowed(){
  if(m_RearMotor.getPosition().getValueAsDouble() >= Constants.Elevator.kStowed * 0.95){
    return true;
  }
  else{
    return false;
  }
}

public void checkAndFix(){
  if(atStowed() == false){
    Stow();
    new WaitCommand(0.25).schedule();
  }
}
  
//Function to set the power of the elevator
  public void SetPower(double power)
    {
      m_FrontMotor.set(-power);
      m_RearMotor.set(power);
  }

  public void Reset(){
    m_RearMotor.set(-0.2);
  }


  //Functions for manual incrementing of the elevator
  public void IncrementIncrease(){
    setPosition(m_RearMotor.getPosition().getValueAsDouble() + 0.5);
  }

  public void IncrementDecrease(){
    setPosition(m_RearMotor.getPosition().getValueAsDouble() - 0.5);
  }

  public void ResetEncoders()
  {
    m_FrontMotor.setPosition(0);
    m_RearMotor.setPosition(0);
  }

  public Command RunCurrentZeroing() {
    return this.run(() -> this.SetPower(-.10))
        .until(() -> m_FrontMotor.getStatorCurrent().refresh().getValueAsDouble() > 40.0)
        .andThen(() -> this.SetPower(0))
        .finallyDo(() -> this.ResetEncoders());
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator/Front/Pos",  m_FrontMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Rear/Pos",  m_RearMotor.getPosition().getValueAsDouble());
    //SmartDashboard.putNumber("Elevator Speed", m_RearMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator Jacked Up", atStowed() && !ElevatorAtZero());
    
  }
}
  

  