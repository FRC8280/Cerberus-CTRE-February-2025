package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Effector extends SubsystemBase  {
    
    public enum EffectorState {
        None,
        Intaking,
        ManualIntake,
        Scoring,
        Ejecting
      }
    
      public boolean debugSpew = false;
    private EffectorState m_EffectorState;
    private  TalonFX m_EffectorMotor;
    private TalonFX m_algeaArm;

    private CANrange m_FrontSensor;
    private CANrange m_RearSensor;
    double m_FrontDistanceValue;
    double m_RearDistanceValue;

    private Timer m_EffectorTimer;

    VelocityVoltage velocityRequest;

    TalonFXConfiguration talonFXConfigs;
    TalonFXConfigurator TalonFXConfigurator;
    MotorOutputConfigs motorConfigs;

    private static final Slot0Configs EffectorGains = new Slot0Configs();
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    //AlgeaArm configs
    TalonFXConfiguration talonFXConfigsAlgea;
    TalonFXConfigurator TalonFXConfiguratorAlgea;
    MotorOutputConfigs motorConfigsAlgea;

    private static final Slot0Configs AlgaeArmGains = new Slot0Configs();
    final VelocityVoltage m_requestAlgea = new VelocityVoltage(0).withSlot(0);

    public Effector() {
        
        m_EffectorMotor = new TalonFX(Constants.Effector.kMotorCanId);
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 40;
        m_EffectorMotor.getConfigurator().apply(currentLimits);
        
        talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_EffectorMotor.getConfigurator().apply(talonFXConfigs);

        EffectorGains.kS = 0.1; // Add 0.1 V output to overcome static friction
        EffectorGains.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        EffectorGains.kP = 0.11;//0.11; // An error of 1 rps results in 0.11 V output
        EffectorGains.kI = 0; // no output for integrated error
        EffectorGains.kD = 0; // no output for error derivative
        m_EffectorMotor.getConfigurator().apply(EffectorGains);

        //Range sensors
        m_FrontSensor = new CANrange(Constants.Effector.kFrontLaserCanId);//new LaserCan();
        m_RearSensor = new CANrange(Constants.Effector.kRearLaserCanId);
        
        m_EffectorTimer = new Timer();
        m_EffectorState = EffectorState.None;

        /*Algea arm */
       m_algeaArm = new TalonFX(Constants.Effector.kAlgeaMotorCandID);
        CurrentLimitsConfigs currentLimitsAlgea = new CurrentLimitsConfigs();
        currentLimitsAlgea.SupplyCurrentLimitEnable = true;
        currentLimitsAlgea.SupplyCurrentLimit = 30;
        m_algeaArm.getConfigurator().apply(currentLimitsAlgea);

        talonFXConfigsAlgea = new TalonFXConfiguration();
        talonFXConfigsAlgea.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_algeaArm.getConfigurator().apply(talonFXConfigsAlgea);

        /*AlgaeArmGains.kS = 0.1; // Add 0.1 V output to overcome static friction
        AlgaeArmGains.kV = 0.06; // A velocity target of 1 rps results in 0.12 V output
        AlgaeArmGains.kA = 0.03;
        AlgaeArmGains.kG = 1.0;

        AlgaeArmGains.kP = 5;//0.11; // An error of 1 rps results in 0.11 V output
        AlgaeArmGains.kI = 0; // no output for integrated error
        AlgaeArmGains.kD = 0.1; // no output for error derivative*/
        //AlgaeArmGains.kS = 0.1; // Add 0.1 V output to overcome static friction
        //AlgaeArmGains.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output

        AlgaeArmGains.kS = 0.1; // Add 0.1 V output to overcome static friction
        AlgaeArmGains.kV = 0.06; // A velocity target of 1 rps results in 0.12 V output
        AlgaeArmGains.kG = 0.09;
        AlgaeArmGains.kP = 2.4;//0.11; // An error of 1 rps results in 0.11 V output
        AlgaeArmGains.kI = 0.0; // no output for integrated error
        AlgaeArmGains.kD = 0.25; // no output for error derivative

        m_algeaArm.getConfigurator().apply(AlgaeArmGains);
        m_algeaArm.setPosition(0); 
    }

    //New arm attachment to effectorn
    public void ResetAlgeaArm()
    {
        final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
        m_algeaArm.setControl(positionRequest.withPosition(0));
        m_EffectorMotor.set(0);
    }

    public boolean aboveThreshold(double value, double constant) {
        return value >= Math.abs(0.95 * constant);
    }
    
    public boolean reachedSetState() {
        double currentPosition = m_algeaArm.getPosition().getValueAsDouble();
        return aboveThreshold(currentPosition, 0);
      }

    public void MoveAlgeaArmAuton()
    {
        //System.out.println("Moving Algea arm Auton version.");
        final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
        m_algeaArm.setControl(positionRequest.withPosition(-1.76));//Constants.Effector.algeaArmScorePosition));
        m_EffectorMotor.set(-50);

        m_EffectorTimer.reset();
        m_EffectorTimer.start();

    }

    public void MoveAlgeaArm()
    {
        //System.out.println("Moving Algea arm.");
        final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
        m_algeaArm.setControl(positionRequest.withPosition(-1.76));//Constants.Effector.algeaArmScorePosition));
        m_EffectorMotor.set(-50);
    }

    public void MoveAlgeaArmNoEffector()
    {
        //System.out.println("Moving Algea arm.");
        final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
        m_algeaArm.setControl(positionRequest.withPosition(-2));//Constants.Effector.algeaArmScorePosition));
    }

    public void StopNewArm()
    {
        final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
        m_algeaArm.setControl(positionRequest.withPosition(0));
        m_EffectorMotor.set(0);
        //System.out.println("Stopping Algea arm.");
    }

    public void RunEffector(double speed) {

        final VelocityVoltage testRequest = new VelocityVoltage(0).withSlot(0);
        m_EffectorMotor.setControl(testRequest.withVelocity(speed));
        //m_EffectorMotor.set(speed);
        m_EffectorTimer.reset();
        m_EffectorTimer.start();
    }

    public void RunEffectorAuton(double speed) {

        final VelocityVoltage testRequest = new VelocityVoltage(0).withSlot(0);
        m_EffectorMotor.setControl(testRequest.withVelocity(speed));
        //m_EffectorMotor.set(speed);
    }

    public void autonIntake(){
        m_EffectorState = EffectorState.Intaking;
        RunEffectorAuton(Constants.Effector.kSpeed);
    }

    public void IntakeCoral()
    {
        m_EffectorState = EffectorState.ManualIntake;
        RunEffector(Constants.Effector.kSpeed);
    }

    public void IntakeCoralSlower()
    {
        m_EffectorState = EffectorState.ManualIntake;
        RunEffector(Constants.Effector.kSlowerSpeed);
    }

    public Boolean IntakeComplete()
    {
        if(m_EffectorState == EffectorState.Intaking && DetectCoralFront())
            return true;
        else
            return false; 
    }

    public boolean Scoring()
    {
        if(m_EffectorState == EffectorState.Scoring)
            return true;
        else
            return false;
    }
    public void ScoreCoral()
    {
        m_EffectorState = EffectorState.Scoring;
        RunEffector(Constants.Effector.kSpeed+10);//20
    }

    public void ScoreCoralTeleOp(double level)
    {
        if(level == Constants.Elevator.levelOne)
        {
            m_EffectorState = EffectorState.Scoring;
            RunEffector(Constants.Effector.kSpeed-8);
        }
        else
        {
            m_EffectorState = EffectorState.Scoring;
            RunEffector(Constants.Effector.kSpeed+10);//20
        }
        
    }

    public void ScoreL1()
    {
        m_EffectorState = EffectorState.Scoring;
        RunEffector(30);
    }

    public void EjectCoral()
    {
        m_EffectorState = EffectorState.Ejecting;
        RunEffector(-Constants.Effector.kSpeed/2);

        m_EffectorTimer.reset();
        m_EffectorTimer.start();
    }

    public void Stop()
    {
        m_EffectorState = EffectorState.None;
        m_EffectorMotor.set(0);
        m_EffectorTimer.stop();
        ResetAlgeaArm();
        //m_EffectorTimer.reset();
    }

    public boolean DetectCoral()
    {    
       if((m_FrontDistanceValue <= Constants.Effector.kCoralDetectionRange) || 
           (m_RearDistanceValue <= Constants.Effector.kCoralDetectionRange))
          return true;
        else
          return false; 
    }

    public boolean DetectCoralRear(){

        if(m_RearDistanceValue <= Constants.Effector.kCoralDetectionRange) 
          return true;
        else
          return false; 
    }

    public boolean DetectCoralFront(){

        if(m_FrontDistanceValue <= Constants.Effector.kCoralDetectionRange) 
          return true;
        else
          return false; 
    }

    public boolean DetectCoralBoth(){

        if((m_FrontDistanceValue <= Constants.Effector.kCoralDetectionRange) && 
           (m_RearDistanceValue <= Constants.Effector.kCoralDetectionRange))
          return true;
        else
          return false;
    }

    @Override
    public void periodic() { 
        m_FrontDistanceValue = m_FrontSensor.getDistance().refresh().getValueAsDouble()*1000;
        m_RearDistanceValue = m_RearSensor.getDistance().refresh().getValueAsDouble()*1000;

        if((m_EffectorState == EffectorState.ManualIntake) && (DetectCoralFront()))
            Stop();
        else if((m_EffectorState == EffectorState.ManualIntake) && DetectCoralRear() )
            IntakeCoralSlower();
        else if(m_EffectorState == EffectorState.Intaking && DetectCoralFront())
            Stop();
        else if(m_EffectorState == EffectorState.Intaking && IntakeComplete())
            Stop();
        else if(m_EffectorState == EffectorState.Scoring && !DetectCoralFront())
            Stop();
        else if(m_EffectorState == EffectorState.Ejecting && !DetectCoralRear())
            Stop();
        //emergency measure
        else if(m_EffectorTimer.isRunning() && m_EffectorTimer.hasElapsed(Constants.Effector.intakeTimerMax))  //emergency shutdown if timer expires
        {
            //System.out.println("Emergency Stop.");
            Stop();
        }   


        SmartDashboard.putNumber("Front Sensor: ", m_FrontDistanceValue);
        SmartDashboard.putNumber("Rear Sensor: ", m_RearDistanceValue);
        //SmartDashboard.putBoolean("CoralDetected", DetectCoral());
        //SmartDashboard.putNumber("Effector Velocity", m_EffectorMotor.getVelocity().getValueAsDouble());
        //SmartDashboard.putBoolean("Effector Timer Running",m_EffectorTimer.isRunning());
        //SmartDashboard.putNumber("Effector Timer Elapsed", m_EffectorTimer.get());
        SmartDashboard.putNumber("Effector Arm Position", m_algeaArm.getPosition().getValueAsDouble());
    }

}
