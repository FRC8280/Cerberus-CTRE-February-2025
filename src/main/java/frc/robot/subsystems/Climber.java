package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {

    private final TalonFX m_ClimbMotor;
    private final TalonFX m_IntakeMotor;

    private final CANcoder absoluteEncoder = new CANcoder(50);

    private final Servo m_RampServo;

    private final double climberOut = -0.067;
    private final double climberIn = 0.113525;


    /**
     * This subsytem that controls the climber.
     */
    public Climber () {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Set CANcoder as the feedback sensor
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackSensorSource = com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackConfigs.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();

        config.Feedback = feedbackConfigs;

        // Set up the climb motor as a brushless motor
        m_ClimbMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID);
        m_IntakeMotor = new TalonFX(ClimberConstants.INTAKE_MOTOR_ID);

        m_ClimbMotor.getConfigurator().apply(config);

        m_RampServo = new Servo(9);
       
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = ClimberConstants.CLIMBER_MOTOR_CURRENT_LIMIT;
        
        // Set the neutral mode to Brake
        m_ClimbMotor.setNeutralMode(NeutralModeValue.Brake);
        m_ClimbMotor.getConfigurator().apply(currentLimits);

        CurrentLimitsConfigs intakeCurrentLimits = new CurrentLimitsConfigs();
        intakeCurrentLimits.SupplyCurrentLimitEnable = true;
        intakeCurrentLimits.SupplyCurrentLimit = ClimberConstants.INTAKE_MOTOR_CURRENT_LIMIT;

        m_IntakeMotor.setNeutralMode(NeutralModeValue.Brake);
        m_IntakeMotor.getConfigurator().apply(intakeCurrentLimits);
    }

    @Override
    public void periodic() {
        //todo check for position and prvent from it from moving too far up or down
        SmartDashboard.putNumber("Ramp Servo Angle", m_RampServo.getAngle());
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

    private void moveToPosition(double rotations) {
        // Use PositionVoltage to request position with voltage control (PIDF)
        final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
        m_ClimbMotor.setControl(positionRequest.withPosition(rotations));
    }

    public void engageClimber(){
        moveToPosition(climberOut);
    }

    public void retractClimber(){
        moveToPosition(climberIn);
    }

}