package frc.robot.subsystems;


import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import java.math.RoundingMode;
import java.text.DecimalFormat;

import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
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
//import com.revrobotics.spark.SparkMaxPIDController;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;


public class Algae extends SubsystemBase {
  
  private SparkFlex m_PivotMotor;
  private SparkClosedLoopController m_pidController;
  private AbsoluteEncoder m_encoder;

  private TalonFX m_Roller;
  private LaserCan m_DistanceSensor;
  private static final Slot0Configs rollerGains = new Slot0Configs();

  public Algae(){
    //useful web page. 
    m_PivotMotor = new SparkFlex (AlgaeConstants.kPivotMotorId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    m_pidController = m_PivotMotor.getClosedLoopController();
    m_encoder = m_PivotMotor.getAbsoluteEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false);
       // .idleMode(IdleMode.kBrake);
    //config.encoder
     //   .positionConversionFactor(1000)
     //   .velocityConversionFactor(1000);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(0.5, 0.0, 0.1);
    
    m_PivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //m_DistanceSensor = new LaserCan(AlgaeConstants.kLaserCanID);
    //m_PivotMotor = (AlgaeConstants.kPivotMotorId);
    m_Roller = new TalonFX(AlgaeConstants.kRollerMotorId);

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

  /*public void setPosition (double setpoint){
    m_PivotMotor.setControl(positionRequest.withPosition(setpoint));
  }*/

  public void setPivot(double setpoint){
    final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    //m_PivotMotor.setControl(positionRequest.withPosition(setpoint));
  }

  public void setVelocity(double velocity){
    final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    m_Roller.setControl(velocityRequest.withVelocity(velocity));
  }

  public void stowPivot(){
    setPivot(AlgaeConstants.k_pivotAngleStow);
  }

  public void scorePivot(){
    setPivot(AlgaeConstants.k_pivotAngleProcessor);
  }

  public void groundPivot(){
    setPivot(AlgaeConstants.k_pivotAngleGround);
  }

  public void intakeAlgae(){
    setVelocity(AlgaeConstants.k_intakeSpeed);
  }

  public void ejectAlgae(){
    setVelocity(AlgaeConstants.k_EjectSpeed);
  }



@Override
  public void periodic() {

    m_pidController.setReference(0.1, ControlType.kPosition);
    //SmartDashboard.putNumber("AlgaePivot/Distance",   m_DistanceSensor.getMeasurement().distance_mm);
    SmartDashboard.putNumber("AlgaePivot Position",   m_encoder.getPosition());

    // This method will be called once per scheduler run
    /*SmartDashboard.putNumber("AlgaePivot/Pos",  m_PivotMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("AlgaePivot/CLO",  m_PivotMotor.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("AlgaePivot/Ouptut",  m_PivotMotor.get());
    SmartDashboard.putBoolean("AlgaePivot/Inverted",  m_PivotMotor.getInverted());
    SmartDashboard.putNumber("AlgaePivot/Current",  m_PivotMotor.getSupplyCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Roller/CLO",  m_Roller.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Roller/Ouptut",  m_Roller.get());
    SmartDashboard.putBoolean("Roller/Inverted",  m_Roller.getInverted());
    SmartDashboard.putNumber("Roller/Right/Current",  m_Roller.getSupplyCurrent().getValueAsDouble());*/

  }
}
