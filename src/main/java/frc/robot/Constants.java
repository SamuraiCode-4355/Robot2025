package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

  public static class SwerveConts {

    public static final int KFLDriveID = 1;
    public static final int KBLDriveID = 3;
    public static final int KFRDriveID = 5;
    public static final int KBRDriveID = 7;

    public static final int KFLTurnID = 2;
    public static final int KBLTurnID = 4;
    public static final int KFRTurnID = 6;
    public static final int KBRTurnID = 8;

    public static final int KFLEncoderID = 21;
    public static final int KBLEncoderID = 22;
    public static final int KFREncoderID = 23;
    public static final int KBREncoderID = 24;

    public static final boolean KFLDriveInverted = false;
    public static final boolean KBLDriveInverted = false;
    public static final boolean KFRDriveInverted = true;
    public static final boolean KBRDriveInverted = false;

    public static final boolean KFLTurnInverted = false;
    public static final boolean KBLTurnInverted = false;
    public static final boolean KFRTurnInverted = false;
    public static final boolean KBRTurnInverted = false;

    public static final boolean KFLEncoderInverted = false;
    public static final boolean KBLEncoderInverted = false;
    public static final boolean KFREncoderInverted = false;
    public static final boolean KBREncoderInverted = false;

    public static final int KGiroID = 25;

    public static final double K_P_PID_Drive = 0.45;
    public static final double K_I_PID_Drive = 0.15; 
    public static final double K_D_PID_Drive = 0.001;

    public static final double K_P_PID_Turn = 0.003;
  } 

  public static class RobotConst {

    public static final double wheelRadio = 0.05;
    public static final double wheelLenght = 2 * Math.PI * wheelRadio;
    public static final double wheelTransmision = 6.85;
    public static final double distanceFront_Back = 0.74;
    public static final double distanceLeft_Right = 0.74;

    private static final double maxSpeedMPerSec = 4.5;
    private static final double powerPercent = 65;// Porcentaje de la velocidad
    public static final double power = maxSpeedMPerSec * (powerPercent / 100);

    public static final Translation2d m_FL_Location = new Translation2d(distanceFront_Back / 2,
         distanceLeft_Right / 2);
    public static final Translation2d m_FR_Location = new Translation2d(distanceFront_Back / 2,
        -distanceLeft_Right / 2);
    public static final Translation2d m_BL_Location = new Translation2d(-distanceFront_Back / 2,
         distanceLeft_Right / 2);
    public static final Translation2d m_BR_Location = new Translation2d(-distanceFront_Back / 2,
        -distanceLeft_Right / 2);

    public static final int kLedPort = 0;
  }
}
