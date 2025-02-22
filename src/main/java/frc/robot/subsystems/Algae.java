package frc.robot.subsystems;


import au.grapplerobotics.LaserCan;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.AlgaeConstants;


public class Algae extends SubsystemBase {
  
  private SparkFlex m_PivotMotor;
  private SparkClosedLoopController m_pidController;
  private AbsoluteEncoder m_encoder;

  private TalonFX m_Roller;
  private LaserCan m_DistanceSensor;
  private static final Slot0Configs rollerGains = new Slot0Configs();
  private double currentPivotPosition = AlgaeConstants.kHomePosition;

  public Algae(){
    //useful web page. 
    m_PivotMotor = new SparkFlex (AlgaeConstants.kPivotMotorId, MotorType.kBrushless);
    m_pidController = m_PivotMotor.getClosedLoopController();
    m_encoder = m_PivotMotor.getAbsoluteEncoder();
    SparkMaxConfig config = new SparkMaxConfig();
    
    config
        .inverted(false)
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .outputRange(-.5, .5)
        .pid(0.5, 0.0, 0.1);
    
    m_PivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //m_DistanceSensor = new LaserCan(AlgaeConstants.kLaserCanID);
    //m_PivotMotor = (AlgaeConstants.kPivotMotorId);
    m_Roller = new TalonFX(AlgaeConstants.kRollerMotorId);

    //TalonFXConfiguration configRoller = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 40;
    m_Roller.getConfigurator().apply(currentLimits);

    rollerGains.kP = 0.11;
    rollerGains.kI = 0;
    rollerGains.kD = 0;
    rollerGains.kS = 0.1;
    rollerGains.kV = 0.12;
    m_Roller.getConfigurator().apply(rollerGains);
  }

  public void setVelocity(double velocity){
    final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    m_Roller.setControl(velocityRequest.withVelocity(velocity));
  }

  public void stowPivot(){
    currentPivotPosition = AlgaeConstants.kHomePosition;
  }

  public void scorePivot(){
    currentPivotPosition = AlgaeConstants.kScoringPosition;
  }

  public void GroundPivot(){
    currentPivotPosition = AlgaeConstants.kIntakePosition;
  }

  public void intakeAlgae(){
    setVelocity(AlgaeConstants.k_intakeSpeed);
  }

  public void ejectAlgae(){
    setVelocity(AlgaeConstants.k_EjectSpeed);
  }



@Override
  public void periodic() {

    //m_pidController.setReference(0.1, ControlType.kPosition);
    //SmartDashboard.putNumber("AlgaePivot/Distance",   m_DistanceSensor.getMeasurement().distance_mm);
    SmartDashboard.putNumber("AlgaePivot Position",   m_encoder.getPosition());


  }
}
