package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class SubElev extends SubsystemBase {

  private static SubElev m_Instance;

  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;
  private SparkMax m_shooterMotor;

  private SparkMaxConfig m_leftConfig;
  private SparkMaxConfig m_rightConfig;
  private SparkMaxConfig m_shooterConfig;

  private DigitalInput m_photoSensor;
  private PIDController m_pid;

  private boolean m_EnabledPID;
  private double m_Output;

  //-----------------------MÉTODO CONSTRUCTOR--------------------------

  public SubElev() {
    
    m_leftMotor = new SparkMax(ElevatorConstants.kLeftID, MotorType.kBrushless);
    m_rightMotor = new SparkMax(ElevatorConstants.kRightID, MotorType.kBrushless);
    m_shooterMotor = new SparkMax(ElevatorConstants.kShooterID, MotorType.kBrushless);

    m_leftConfig = new SparkMaxConfig();
    m_rightConfig = new SparkMaxConfig();
    m_shooterConfig = new SparkMaxConfig();

    m_photoSensor = new DigitalInput(ElevatorConstants.kPhotoPort);
    
    m_pid = new PIDController(ElevatorConstants.kP_PID, ElevatorConstants.kI_PID, ElevatorConstants.kD_PID);
    m_pid.setTolerance(0.25);

    m_leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.kElevLimitCurrent);
    m_rightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.kElevLimitCurrent);
    m_shooterConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.kShooterLimitCurrent);

    m_leftMotor.clearFaults();
    m_rightMotor.clearFaults();
    m_shooterMotor.clearFaults();
    
    m_leftMotor.configure(m_leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(m_rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shooterMotor.configure(m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //---------------------MÉTODO DE FABRICA------------------------------

  public static SubElev getInstance(){

    if(m_Instance == null){

      m_Instance = new SubElev();
    }
    return m_Instance;
  }

  //-----------------------ATRIBUTOS-------------------------------------

  public boolean atSetPoint(){

    return m_pid.atSetpoint();
  }

  public boolean coral(){

    return m_photoSensor.get();
  }  

  public double encoderElev(){

    return SubClimber.getInstance().getEncoderElev();
  }

  //----------------------MÉTODOS------------------------------------------

  public void suction(){

    m_shooterMotor.set(ElevatorConstants.kSuctionPower);
  }

  public void stopElev(){

    m_leftMotor.set(0.0);
    m_rightMotor.set(0.0);
  }

  public void shoot(double power){

    m_shooterMotor.set(power);
  }

  public void stopShoot(){

    m_shooterMotor.set(0.0);
  }

  public void enabledPID(boolean enable){

    m_EnabledPID = enable;
  }

  public void setLevel(int level){

    switch(level){

      case 0:
        m_pid.setTolerance(0.8);
        m_pid.setSetpoint(ElevatorConstants.kLevel0);
      break;

      case 1:
        m_pid.setTolerance(0.25);  
        m_pid.setSetpoint(ElevatorConstants.kLevel1);       
      break;

      case 2:
        m_pid.setSetpoint(ElevatorConstants.kLevel2);
        m_pid.setTolerance(0.25);
        //m_MaxOutput = ElevatorConstants.maximumPower;   
      break;

      case 3:
        m_pid.setSetpoint(ElevatorConstants.kLevel3);
        m_pid.setTolerance(0.25);
      break;

      default:
        m_EnabledPID = false;
      break;
    }
  }

 /*  public void setIdleModeElev(boolean Break){

    m_leftConfig.idleMode(Break ? IdleMode.kBrake : IdleMode.kCoast);
    m_rightConfig.idleMode(Break ? IdleMode.kBrake : IdleMode.kCoast);

    m_leftMotor.configure(m_leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(m_rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }*/


  //----------------------MÉTODO PERIODICO----------------------------

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Encoder Elev", SubClimber.getInstance().getEncoderElev());

    SmartDashboard.putBoolean("Coral", coral());
    SmartDashboard.putBoolean("AtSetPointElev", atSetPoint());

    m_Output = m_pid.calculate(SubClimber.getInstance().getEncoderElev());

    if(m_Output > ElevatorConstants.maximumPower)
      m_Output = ElevatorConstants.maximumPower;

    if(m_Output < ElevatorConstants.maximumPower * -1)
      m_Output = ElevatorConstants.maximumPower * -1;

    if(m_EnabledPID){

      m_leftMotor.set(m_Output);
      m_rightMotor.set(-m_Output);
    }
    else{

      m_leftMotor.set(0.0);
      m_rightMotor.set(0.0);
    }
  }
}
