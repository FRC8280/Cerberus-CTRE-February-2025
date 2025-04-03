package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {

    private final TalonFX m_ClimbMotor;

    private final Servo m_RampServo;
    private final Servo m_FootServo;

    /**
     * This subsytem that controls the climber.
     */
    public Climber () {

        // Set up the climb motor as a brushless motor
        m_ClimbMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID);
        
        m_RampServo = new Servo(9);
        m_FootServo = new Servo(8);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = ClimberConstants.CLIMBER_MOTOR_CURRENT_LIMIT;
        
        // Set the neutral mode to Brake
        m_ClimbMotor.setNeutralMode(NeutralModeValue.Brake);
        m_ClimbMotor.getConfigurator().apply(currentLimits);
    }

    @Override
    public void periodic() {
        //todo check for position and prvent from it from moving too far up or down
        SmartDashboard.putNumber("Ramp Servo Angle", m_RampServo.getAngle());
        SmartDashboard.putNumber("Foot Servo Angle", m_FootServo.getAngle());
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

    public void ArmClimber(){
        m_RampServo.setAngle(180);
        m_FootServo.setAngle(80);
    }

    public void ResetClimber(){
        m_RampServo.setAngle(00);
        m_FootServo.setAngle(149);
    }

}