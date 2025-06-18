package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DistanceConstants;

public class DistanceSensorSystem extends SubsystemBase {

    public double leftCompare = 1;
    public double centerCompare = 1;
    public double rightCompare =1 ;
    public final CANrange[] m_alignRanges;
    double[] alignValues;

    public boolean CloseEnoughToReef()
    {
       // if( (alignValues[0] <= Constants.DistanceConstants.reefAlignedDistance) && (alignValues[1] <= Constants.DistanceConstants.reefAlignedDistance))
       if( (m_alignRanges[0].getDistance().refresh().getValueAsDouble() <= Constants.DistanceConstants.reefAlignedDistance) || (m_alignRanges[1].getDistance().refresh().getValueAsDouble() <= Constants.DistanceConstants.reefAlignedDistance))
            return true;
        else
            return false;
    }

    public double LongestDistance()
    {
        double firstValue = m_alignRanges[0].getDistance().refresh().getValueAsDouble();
        double secondValue = m_alignRanges[1].getDistance().refresh().getValueAsDouble();
        if(firstValue >= secondValue)
        return firstValue;
    else
        return secondValue;
    }
    
    public DistanceSensorSystem() {
        m_alignRanges = new CANrange[]{
        new CANrange(DistanceConstants.leftAlignmentRangeId),
        new CANrange(DistanceConstants.righttAlignmentRangeId)
        };

       alignValues = new double[m_alignRanges.length];
    }

    public void updateAlignValues() {
        for (int i = 0; i < m_alignRanges.length; i++) {
            alignValues[i] = m_alignRanges[i].getDistance().refresh().getValueAsDouble();
        }
    }
    


    @Override
    public void periodic() {
       updateAlignValues();   
       for (int i = 0; i < alignValues.length; i++) {
            SmartDashboard.putNumber("Align Value " + i, alignValues[i]);
        }
        
    }
    
}
