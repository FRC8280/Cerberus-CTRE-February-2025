package frc.robot.subsystems;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {

    private final Servo m_RampServo;
    private final TalonFX m_ClimbMotor;
    private final TalonFX m_IntakeMotor;
    private final CANcoder m_ThroughBoreEncoder;
    private final CANrange m_ClimberRange;

    private final double canCoderZero = +0.1067;
    private final double climberOut = -0.067;
    private final double climberIn = 0.145;//0.113525;
    private final double maxExtension = 0.145;//0.153;
    private final double minExtension = -0.059;
    
    private static final double kP = 350.0;   // Position loop P (Volts/rotation error with PositionVoltage)
    private static final double kI = 5;    // Integral gain
    private static final double kD = 2;    // Derivative gain
    private static final double kS = 12.5+2;    // Static friction feedforward (Volts)
    private static final double gravityCompensation = 2;


   /**
   * If you want to command in degrees, one "turn" = 360 deg.
   * We'll issue position targets in "rotations" of the mechanism.
   */
    private static final double DEGREES_PER_TURN = 360.0;


    /**
     * This subsytem that controls the climber.
     */
    public Climber () {

        m_ClimberRange = new CANrange(34);

        //Setup servo for tray
        m_RampServo = new Servo(9);

        //Setup Intake motor on climber. 
        m_IntakeMotor = new TalonFX(ClimberConstants.INTAKE_MOTOR_ID);
        CurrentLimitsConfigs intakeCurrentLimits = new CurrentLimitsConfigs();
        intakeCurrentLimits.SupplyCurrentLimitEnable = true;
        intakeCurrentLimits.SupplyCurrentLimit = ClimberConstants.INTAKE_MOTOR_CURRENT_LIMIT;

        m_IntakeMotor.setNeutralMode(NeutralModeValue.Brake);
        m_IntakeMotor.getConfigurator().apply(intakeCurrentLimits);

        //Setup cancoder
        m_ThroughBoreEncoder  = new CANcoder(50);  
        
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = canCoderZero;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // Or CounterClockwise_Positive
        m_ThroughBoreEncoder.getConfigurator().apply(canCoderConfig);

        // Configure the climb motor with PID and feedforward
        m_ClimbMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS; // Static friction feedforward

        //Setup the motor to use the cancoder
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackSensorSource = com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackConfigs.FeedbackRemoteSensorID = m_ThroughBoreEncoder.getDeviceID();
        config.Feedback = feedbackConfigs;

        m_ClimbMotor.getConfigurator().apply(config);  //Apply the settings

        // Set up the climb motor as a brushless motor       
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 40.0; // Increase this value if needed
        currentLimits.SupplyCurrentLimit = ClimberConstants.CLIMBER_MOTOR_CURRENT_LIMIT;
        
        // Set the neutral mode to Brake
        m_ClimbMotor.setNeutralMode(NeutralModeValue.Brake);
        m_ClimbMotor.getConfigurator().apply(currentLimits);
    }

    @Override
    public void periodic() {
        double canRangeDistance =  m_ClimberRange.getDistance().refresh().getValueAsDouble();
        SmartDashboard.putNumber("Climger  Distance", canRangeDistance);
        SmartDashboard.putBoolean("Climber Captured", canRangeDistance < 0.5);

        //todo check for position and prvent from it from moving too far up or down
        //SmartDashboard.putNumber("Ramp Servo Angle", m_RampServo.getAngle());

        // Display the position of the CANcoder
         //SmartDashboard.putNumber("CANcoder Position", m_ThroughBoreEncoder.getPosition().getValueAsDouble());

        // Display the position of the motor's internal encoder
       // SmartDashboard.putNumber("Motor Internal Encoder Position", m_ClimbMotor.getPosition().getValueAsDouble());
    }

    public boolean AtMaxExtensions()
    {
        if(m_ThroughBoreEncoder.getPosition().getValueAsDouble()>=maxExtension)
            return true;
        else 
            return false;
    }
    public boolean AtMaxContraction()
    {
        if(m_ThroughBoreEncoder.getPosition().getValueAsDouble() <=minExtension)
            return true;
        else
            return false;
    }

    /**
     * Use to run the climber, can be set to run from 100% to -100%.
     * Keep in mind that the direction changes based on which way the winch is wound.
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runClimber(double speed){
        m_ClimbMotor.set(speed);
    }

    /**
     * Runs the intake motor at 15% power.
     */
    
    public void IntakeCage() {
        m_IntakeMotor.set(-0.15);
    }

    /**
     * Stops the intake motor by settinAg power to 0%.
     */
    public void StopIntakeCage() {
        m_IntakeMotor.set(0.0);
    }

    public void ArmClimber(){
        m_RampServo.setAngle(180);
    }

    public void ResetClimber(){
        m_RampServo.setAngle(00);
    }

      /** Convenience: command a position in DEGREES of the mechanism. */
    public void setTargetDegrees(double mechanismDegrees) {
        m_ClimbMotor.setControl(new PositionVoltage(mechanismDegrees / DEGREES_PER_TURN).withSlot(0));
        //.withFeedForward(kS+gravityCompensation));
    }

    public void moveToPosition(double rotations) {
        // Use PositionVoltage to request position with voltage control (PIDF)
        final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
        m_ClimbMotor.setControl(positionRequest.withPosition(rotations));
    }

    public void retractClimber(){
        moveToPosition(climberOut);
    }

    public void engageClimber(){
        moveToPosition(climberIn);
    }



}