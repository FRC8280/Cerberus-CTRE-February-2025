package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DistanceConstants;



public class DistanceSensorSystem extends SubsystemBase {
    public final CANrange[] m_alignRanges;
    double[] alignValues;
   
    public final CANrange[] m_TargetArray;
    double[] m_TargetArrayValues;

    

    public DistanceSensorSystem() {
        m_alignRanges = new CANrange[]{
        new CANrange(DistanceConstants.leftAlignmentRangeId),
        new CANrange(DistanceConstants.righttAlignmentRangeId)
        };

        alignValues = new double[m_alignRanges.length];

        m_TargetArray = new CANrange[] {
            new CANrange(DistanceConstants.TargetArray0),
            new CANrange(DistanceConstants.TargetArray1),
            new CANrange(DistanceConstants.TargetArray2),
            new CANrange(DistanceConstants.TargetArray3),
            new CANrange(DistanceConstants.TargetArray4)
        };

        m_TargetArrayValues = new double[m_TargetArray.length];

    }

    public void updateAlignValues() {
        for (int i = 0; i < m_alignRanges.length; i++) {
            alignValues[i] = m_alignRanges[i].getDistance().refresh().getValueAsDouble() * 1000;
        }
    }

    public void updateTargetArrayValues() {
        for (int i = 0; i < m_TargetArray.length; i++) {
            m_TargetArrayValues[i] = m_TargetArray[i].getDistance().refresh().getValueAsDouble() * 1000;
        }
    }


    @Override
    public void periodic() {
        updateAlignValues();
        updateTargetArrayValues();

        
        for (int i = 0; i < alignValues.length; i++) {
            SmartDashboard.putNumber("Align Value " + i, alignValues[i]);
        }

        for (int i =0; i < m_TargetArrayValues.length; i++) {
            SmartDashboard.putNumber("Pole Value " + i, m_TargetArrayValues[i]);
        }
    }
    
}
