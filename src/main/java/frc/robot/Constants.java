package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

  public static class SwerveConstants {

    public static final byte kFLDriveID = 1;
    public static final byte kFRDriveID = 3;
    public static final byte kBLDriveID = 5;
    public static final byte kBRDriveID = 7;

    public static final byte kFLTurnID = 2;
    public static final byte kFRTurnID = 4;
    public static final byte kBLTurnID = 6;
    public static final byte kBRTurnID = 8;

    public static final byte kFLEncoderID = 21;
    public static final byte kFREncoderID = 22;
    public static final byte kBLEncoderID = 23;
    public static final byte kBREncoderID = 24;

    public static final boolean kFLDriveInverted = false;
    public static final boolean kFRDriveInverted = true;
    public static final boolean kBLDriveInverted = true;
    public static final boolean kBRDriveInverted = false;

    public static final boolean kFLTurnInverted = false;
    public static final boolean kFRTurnInverted = false;
    public static final boolean kBLTurnInverted = false;
    public static final boolean kBRTurnInverted = false;

    public static final boolean kFLEncoderInverted = false;
    public static final boolean kFREncoderInverted = false;
    public static final boolean kBLEncoderInverted = false;
    public static final boolean kBREncoderInverted = false;

    public static final byte kGyroID = 25;

    public static final float kP_PID_Drive = 0.45f;
    public static final float kI_PID_Drive = 0.15f; 
    public static final float kD_PID_Drive = 0.001f;

    public static final float kP_PID_Turn = 0.003f;

    public static final byte kLimitCurrentDrive = 35;
    public static final byte kLimitCurrentTurn = 20;
  } 

  public static class RobotConstants {

    public static final boolean redAlliance = false;

    public static final float kWheelRadio = 0.05f;
    public static final double kWheelLenght = 2 * Math.PI * kWheelRadio;
    public static final float kWheelTransmision = 6.75f;
    public static final float kDistanceFront_Back = 0.74f;
    public static final float kDistanceLeft_Right = 0.74f;

    private static final float kMaxSpeedMPerSec = 4.5f;
    private static final float kPowerPercent = 65;// Porcentaje de la velocidad
    public static final float kPower = kMaxSpeedMPerSec * (kPowerPercent / 100);

    public static final Translation2d kFL_Location = new Translation2d(kDistanceFront_Back / 2,
         kDistanceLeft_Right / 2);
    public static final Translation2d kFR_Location = new Translation2d(kDistanceFront_Back / 2,
        -kDistanceLeft_Right / 2);
    public static final Translation2d kBL_Location = new Translation2d(-kDistanceFront_Back / 2,
         kDistanceLeft_Right / 2);
    public static final Translation2d kBR_Location = new Translation2d(-kDistanceFront_Back / 2,
        -kDistanceLeft_Right / 2);

    public static final byte kLedPort = 0;
  }

  public static class ShooterConstants{

    public static final byte kShooterID = 9;
  }

  public static class FieldConstants{

   // public static final float[] kReefCenter = Conversions.coordinates(new float[]{-4.2f, 0.0f});

    public static final float kTxLeft = 7.4f;
    public static final float kTaLeft = 3.3f;

    public static final float kTxRight = -15.4f;
    public static final float kTaRight = 3.7f;

    public static final float kDistanceFront = 0.44f;//36
  }
}
