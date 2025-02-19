package frc.robot.math;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;

public class Conversions {

    public static double encToDegrees(double value, boolean inverted){

        value *= 360;
        value *= inverted ? -1 : 1;
        return value;
    }

    public static double RPMToMPerSec(double RPM){

        double wheelRPM = RPM / RobotConstants.kWheelTransmision;
        double wheelRadPerSec = wheelRPM * (2 * Math.PI / 60);
        return wheelRadPerSec * RobotConstants.kWheelRadio;
    }

    public static double driveTurnsToM(double turns){

        double turnsWheel = turns / RobotConstants.kWheelTransmision;
        double meters = turnsWheel * RobotConstants.kWheelLenght;
        return meters;
    }

    public static double gyroReverse(double heading){

        double headingRev = heading += (heading < 0) ? 180 : -180;
        return headingRev; 
    }

    public static float[] coordinates(float[] coordinates){

        if(FieldConstants.redAlliance){

            coordinates[0] *= -1;
            coordinates[1] *= -1;    
        }
        return coordinates;
    }
}
