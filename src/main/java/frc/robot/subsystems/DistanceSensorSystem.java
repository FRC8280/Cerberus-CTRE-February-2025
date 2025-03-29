package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DistanceConstants;



public class DistanceSensorSystem extends SubsystemBase {

    public enum ReefPoleAlignment {
        FAR_LEFT,
        LEFT,
        CENTER,
        RIGHT,
        FAR_RIGHT,
        NOT_FOUND
    }

    public final CANrange[] m_alignRanges;
    double[] alignValues;
   
    public final CANrange[] m_TargetArray;
   double[] m_TargetArrayValues;
   double[] m_TargetArrayValuesStd;

    public boolean CloseEnoughToReef()
    {
       // if( (alignValues[0] <= Constants.DistanceConstants.reefAlignedDistance) && (alignValues[1] <= Constants.DistanceConstants.reefAlignedDistance))
       if( (m_alignRanges[0].getDistance().refresh().getValueAsDouble() <= Constants.DistanceConstants.reefAlignedDistance) && (m_alignRanges[1].getDistance().refresh().getValueAsDouble() <= Constants.DistanceConstants.reefAlignedDistance))
            return true;
        else
            return false;
    }

    public boolean ReefScoreAligned()
    {
        //if(m_TargetArrayValues[2] <= Constants.DistanceConstants.reefScoringDistance)
        if(m_TargetArray[2].getDistance().refresh().getValueAsDouble() <= Constants.DistanceConstants.reefScoringDistance)
            return true;
        else   
            return false;
    }
    
    public int getIndexOfLowestValue() {
        double lowestValue = Double.MAX_VALUE; // Initialize with the maximum possible value
        int lowestIndex = -1; // Initialize with an invalid index
    
        for (int i = 0; i < m_TargetArray.length; i++) {
            double currentValue = m_TargetArray[i].getDistance().refresh().getValueAsDouble();
            if (currentValue < lowestValue) {
                lowestValue = currentValue;
                lowestIndex = i;
            }
        }
    
        return lowestIndex; // Return the index of the lowest value
    }

    public ReefPoleAlignment LocateReefPole()
    {

        int lowIndex = getIndexOfLowestValue();
        switch (lowIndex)
        {
            case 0:
                return ReefPoleAlignment.FAR_LEFT;
            case 1:
                return ReefPoleAlignment.LEFT;
            case 2:
                return ReefPoleAlignment.CENTER;
            case 3:
                return ReefPoleAlignment.RIGHT;
            case 4:
                return ReefPoleAlignment.FAR_RIGHT;
            default:
                return ReefPoleAlignment.NOT_FOUND;
        }

        /*if( m_TargetArray[0].getDistance().refresh().getValueAsDouble() <= Constants.DistanceConstants.reefDetectionThreshold)
            return ReefPoleAlignment.FAR_LEFT;
        else if( m_TargetArray[1].getDistance().refresh().getValueAsDouble() <= Constants.DistanceConstants.reefDetectionThreshold)
            return ReefPoleAlignment.LEFT;
        else if( m_TargetArray[2].getDistance().refresh().getValueAsDouble() <= Constants.DistanceConstants.reefScoringDistance)
            return ReefPoleAlignment.CENTER;
        else if( m_TargetArray[3].getDistance().refresh().getValueAsDouble() <= Constants.DistanceConstants.reefDetectionThreshold)
            return ReefPoleAlignment.RIGHT;
        else if( m_TargetArray[4].getDistance().refresh().getValueAsDouble() <= Constants.DistanceConstants.reefDetectionThreshold)
            return ReefPoleAlignment.FAR_RIGHT;
        
        return ReefPoleAlignment.FAR_RIGHT;*/
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

        m_TargetArray = new CANrange[] {
            new CANrange(DistanceConstants.TargetArray0),
            new CANrange(DistanceConstants.TargetArray1),
            new CANrange(DistanceConstants.TargetArray2),
            new CANrange(DistanceConstants.TargetArray3),
            new CANrange(DistanceConstants.TargetArray4)
        };

       m_TargetArrayValues = new double[m_TargetArray.length];
       m_TargetArrayValuesStd = new double[m_TargetArray.length];

    }

    public void updateAlignValues() {
        for (int i = 0; i < m_alignRanges.length; i++) {
            alignValues[i] = m_alignRanges[i].getDistance().refresh().getValueAsDouble();
        }
    }
    
    public void updateTargetArrayValues() {
        for (int i = 0; i < m_TargetArray.length; i++) {
            m_TargetArrayValues[i] = m_TargetArray[i].getDistance().refresh().getValueAsDouble();
            m_TargetArrayValuesStd[i] = m_TargetArray[i].getDistanceStdDev(true).getValueAsDouble();
        }
    }


    @Override
    public void periodic() {
       //updateAlignValues();
       //updateTargetArrayValues();

      /*
       for (int i = 0; i < alignValues.length; i++) {
            SmartDashboard.putNumber("Align Value " + i, alignValues[i]);
        }

        for (int i =0; i < m_TargetArrayValues.length; i++) {
            SmartDashboard.putNumber("Array Value " + i, m_TargetArrayValues[i]);
        } 

        for (int i =0; i < m_TargetArrayValues.length; i++) {
            SmartDashboard.putNumber("StdDev Value " + i, m_TargetArrayValuesStd[i]);
        }*/ 
        
    }
    
}
