package frc.robot.math;

import frc.robot.Constants.RobotConst;

public class Conversions {

    public static double encToDegrees(double value, boolean inverted){

        value*= 360;

        if(inverted){
            value*= -1;
        }
        
        return value;
    }

    public static double RPMToMPerSec(double RPM){

        double wheelRPM = RPM / RobotConst.wheelTransmision;
        double wheelRadPerSec = wheelRPM * (2 * Math.PI / 60);
        return wheelRadPerSec * RobotConst.wheelRadio;
    }

    public static double driveTurnsToM(double turns){

        double turnsWheel = turns / RobotConst.wheelTransmision;
        double meters = turnsWheel * RobotConst.wheelLenght;
        return meters;
    }

    public static double gyroReverse(double heading){

        double headingRev = heading += (heading < 0) ? 180 : -180;
        return headingRev; 
    }
}
