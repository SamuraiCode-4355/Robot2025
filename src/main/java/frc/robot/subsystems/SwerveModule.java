package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.math.Configure;
import frc.robot.math.Conversions;

public class SwerveModule {

    private SparkMax m_Drive;
    private SparkMax m_Turn;
    private SparkMaxConfig mDriveConfig;
    private SparkMaxConfig mTurnConfig;

    private boolean m_EncInv;

    private CANcoder m_Encoder;

    private PIDController m_DrivePID;
    private PIDController m_TurnPID;

    private SwerveModuleState m_CurrentState;
    private SwerveModuleState m_DesiredState;

    private double setPointDrive;
    private double setPointTurn;

    private double velDrive;
    private double velTurn;
    private byte encoderID;

    //------------------MÉTODO CONSTRUCTOR--------------------------------

    public SwerveModule(byte DriveID, byte TurnID, byte EncoderID,
        boolean DriveInv, boolean TurnInv, boolean EncoderInv){

        m_Drive = new SparkMax(DriveID, MotorType.kBrushless);
        m_Turn = new SparkMax(TurnID, MotorType.kBrushless);

        this.encoderID = EncoderID;

        mDriveConfig = new SparkMaxConfig();
        mTurnConfig = new SparkMaxConfig();

        mDriveConfig.inverted(DriveInv).smartCurrentLimit(SwerveConstants.kLimitCurrentDrive);
        mTurnConfig.inverted(TurnInv).smartCurrentLimit(SwerveConstants.kLimitCurrentTurn);

        m_Drive.configure(mDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_Turn.configure(mTurnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.m_EncInv = EncoderInv;
                
        m_DrivePID = new PIDController(SwerveConstants.kP_PID_Drive, SwerveConstants.kI_PID_Drive, SwerveConstants.kD_PID_Drive);
        m_TurnPID = new PIDController(SwerveConstants.kP_PID_Turn, 0, 0);

        m_TurnPID.setTolerance(0.2);
        m_TurnPID.enableContinuousInput(0, 180);

        m_CurrentState = new SwerveModuleState();
        m_DesiredState = new SwerveModuleState();
    }

    //-----------------------ATRIBUTOS-----------------------------

    private double EncDegrees(){

        if(m_Encoder == null)
            return 0.0;     
        return Conversions.encToDegrees(m_Encoder.getAbsolutePosition().getValueAsDouble(), m_EncInv);
    }

    private double speedMPerSecond(){

        return Conversions.RPMToMPerSec(m_Drive.getEncoder().getVelocity());
    }

    public double getMetersTraveled(){

        return Conversions.driveTurnsToM(m_Drive.getEncoder().getPosition());
    }

    public SwerveModuleState getCurrentState(){

        m_CurrentState = new SwerveModuleState(speedMPerSecond(), new Rotation2d(Units.degreesToRadians(EncDegrees())));
        return m_CurrentState;
    }

    public SwerveModuleState getDesiredState(){

        m_DesiredState = new SwerveModuleState(setPointDrive, new Rotation2d(Units.degreesToRadians(setPointTurn)));
        return m_DesiredState;
    }

    //---------------------------MÉTODOS-----------------------------------

    public void resetMetersTraveled(){

        m_Drive.getEncoder().setPosition(0.0);
    }

    public void setBreak(boolean Break){

        mDriveConfig.idleMode(Break ? IdleMode.kBrake : IdleMode.kCoast);
        m_Drive.configure(mDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void initEncoder(){

        if(m_Encoder == null){

            m_Encoder = new CANcoder(encoderID);
        }
    }

    public void setDesiredState(SwerveModuleState newState){

        newState.optimize(getCurrentState().angle);
        newState.speedMetersPerSecond *= newState.angle.minus(getCurrentState().angle).getCos();

        setPointDrive = newState.speedMetersPerSecond * RobotConstants.kPower;
        setPointTurn = newState.angle.getDegrees();

        m_DrivePID.setSetpoint(setPointDrive);
        m_TurnPID.setSetpoint(setPointTurn);

        velDrive = m_DrivePID.calculate(speedMPerSecond());
        velTurn = m_TurnPID.calculate(EncDegrees());

        m_Turn.set(velTurn);
        m_Drive.set(Configure.getDrive() ? velDrive : 0.0);
    }  

    public void stop(){

        m_Drive.set(0.0);
        m_Turn.set(0.0);
    }
}