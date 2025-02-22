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

  private LaserCan m_BottomSensor;

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

    m_BottomSensor = new LaserCan(Constants.Elevator.kBottomLaserCanId);

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
    
    elevatorGains.kP = 2.4;
    elevatorGains.kI = 0;
    elevatorGains.kD = 0.25;
    elevatorGains.kG = 0.09;
    elevatorGains.kV = 3.01;
    elevatorGains.kA = 0.01;
    m_RearMotor.getConfigurator().apply(elevatorGains);
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

  //Functions to check if the elevator has reached the set position
  public boolean aboveThreshold(double value, double constant) {
    return value >= 0.95 * constant;
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
    
    /*LaserCan.Measurement elevatorMeasurement = m_BottomSensor.getMeasurement();
    if(elevatorMeasurement.distance_mm > Constants.Elevator.kElevatorDetectionRange && targetElevatorPosition
     == Constants.Elevator.kStowed && reachedSetState() == true){
      Reset();
    }
    else if (elevatorMeasurement.distance_mm <= Constants.Elevator.kElevatorDetectionRange && targetElevatorPosition
     == Constants.Elevator.kStowed && reachedSetState() == true){
      SetPower(0);
      m_RearMotor.setPosition(0);
      m_FrontMotor.setPosition(0);
    }*/
    
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator/Front/Pos",  m_FrontMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Rear/Pos",  m_RearMotor.getPosition().getValueAsDouble());
    //SmartDashboard.putNumber("Elevator Speed", m_RearMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Sensor", m_BottomSensor.getMeasurement().distance_mm);
    
  }
}
  

  