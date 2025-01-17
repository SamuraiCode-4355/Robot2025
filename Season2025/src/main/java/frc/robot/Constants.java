package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

  public static class SwerveConts {

    public static final int kFLDriveID = 3;
    public static final int kFRDriveID = 1;
    public static final int kBLDriveID = 7;
    public static final int kBRDriveID = 5;

    public static final int kFLTurnID = 4;
    public static final int kFRTurnID = 2;
    public static final int kBLTurnID = 8;
    public static final int kBRTurnID = 6;

    public static final int kFLEncoderID = 22;
    public static final int kFREncoderID = 21;
    public static final int kBLEncoderID = 24;
    public static final int kBREncoderID = 23;

    public static final boolean kFLDriveInverted = false;
    public static final boolean kFRDriveInverted = false;
    public static final boolean kBLDriveInverted = false;
    public static final boolean kBRDriveInverted = true;

    public static final boolean kFLTurnInverted = false;
    public static final boolean kFRTurnInverted = false;
    public static final boolean kBLTurnInverted = false;
    public static final boolean kBRTurnInverted = false;

    public static final boolean kFLEncoderInverted = false;
    public static final boolean kFREncoderInverted = false;
    public static final boolean kBLEncoderInverted = false;
    public static final boolean kBREncoderInverted = false;

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

  public static class ShootConstant{

    public static final int KM1 = 10;
  }
}
