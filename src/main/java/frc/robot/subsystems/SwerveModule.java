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
import frc.robot.Constants.RobotConst;
import frc.robot.Constants.SwerveConts;
import frc.robot.math.Conversions;

public class SwerveModule {

    private SparkMax m_Drive;
    private SparkMax m_Turn;
    private SparkMaxConfig mDriveConfig;
    private SparkMaxConfig mTurnConfig;

    private boolean mEncInv;

    private CANcoder m_Encoder;

    private PIDController m_DrivePID;
    private PIDController m_TurnPID;

    private SwerveModuleState actualState;
    private SwerveModuleState desiredState;

    private double setPointDrive;
    private double setPointTurn;

    private double velDrive;
    private double velTurn;

    public SwerveModule(int DriveID, int TurnID,int EncoderID,
         boolean DriveInv, boolean TurnInv, boolean EncoderInv){

        m_Turn = new SparkMax(TurnID, MotorType.kBrushless);
        m_Drive = new SparkMax(DriveID, MotorType.kBrushless);

        mDriveConfig = new SparkMaxConfig();
        mTurnConfig = new SparkMaxConfig();

        mDriveConfig.inverted(DriveInv);
        mTurnConfig.inverted(TurnInv);

        m_Drive.configure(mDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_Turn.configure(mTurnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.mEncInv = EncoderInv;
        
        m_Encoder = new CANcoder(EncoderID);
        
        m_DrivePID = new PIDController(SwerveConts.K_P_PID_Drive, SwerveConts.K_I_PID_Drive, SwerveConts.K_D_PID_Drive);
        m_TurnPID = new PIDController(SwerveConts.K_P_PID_Turn, 0, 0);

        m_DrivePID.setTolerance(0.5);
        m_TurnPID.setTolerance(0.2);

        m_TurnPID.enableContinuousInput(0, 180);

        actualState = new SwerveModuleState();
        desiredState = new SwerveModuleState();
    }

    private double EncDegrees(){

        return Conversions.encToDegrees(m_Encoder.getAbsolutePosition().getValueAsDouble(),mEncInv);
    }

    private double speedMPerSecond(){

        return Conversions.RPMToMPerSec(m_Drive.getEncoder().getVelocity());
    }

    public double getMetersTraveled(){

        return Conversions.driveTurnsToM(m_Drive.getEncoder().getPosition());
    }

    public void setDesiredState(SwerveModuleState newState){

        newState.optimize(getActualState().angle);
        newState.speedMetersPerSecond *= newState.angle.minus(getActualState().angle).getCos();

        setPointDrive = newState.speedMetersPerSecond * RobotConst.power;
        setPointTurn = newState.angle.getDegrees();

        m_DrivePID.setSetpoint(setPointDrive);
        m_TurnPID.setSetpoint(setPointTurn);

        velDrive = m_DrivePID.calculate(speedMPerSecond());
        velTurn = m_TurnPID.calculate(EncDegrees());

        m_Drive.set(velDrive);
        m_Turn.set(velTurn);
    }  

    public SwerveModuleState getActualState(){

        actualState = new SwerveModuleState(speedMPerSecond(), new Rotation2d(Units.degreesToRadians(EncDegrees())));
        return actualState;
    }

    public SwerveModuleState getDesiredState(){

        desiredState = new SwerveModuleState(setPointDrive, new Rotation2d(Units.degreesToRadians(setPointTurn)));
        return desiredState;
    }

    public double getVoltageDrive(){

        return m_Drive.getAppliedOutput();
    }

    public void setBreak(){

        mDriveConfig.idleMode(IdleMode.kBrake);
        m_Drive.configure(mDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setCoast(){

        mDriveConfig.idleMode(IdleMode.kCoast);
        m_Drive.configure(mDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setDrive(double speed){

        m_Drive.set(speed);
    }

    public void stop(){

        m_Drive.set(0.0);
        m_Turn.set(0.0);
    }
}