package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.math.Configure;

public class SubElev extends SubsystemBase {

  private static SubElev instance;

  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;
  private SparkMax m_shooterMotor;

  private SparkMaxConfig m_leftConfig;
  private SparkMaxConfig m_rightConfig;
  private SparkMaxConfig m_shooterConfig;

  private ColorSensorV3 m_colorSensor;

  private PIDController m_pid;
  private boolean enabledPID;
  private double output;

  public SubElev() {
    
    m_leftMotor = new SparkMax(ElevatorConstants.kLeftID, MotorType.kBrushless);
    m_rightMotor = new SparkMax(ElevatorConstants.kRightID, MotorType.kBrushless);
    m_shooterMotor = new SparkMax(ElevatorConstants.kShooterID, MotorType.kBrushless);

    m_leftConfig = new SparkMaxConfig();
    m_rightConfig = new SparkMaxConfig();
    m_shooterConfig = new SparkMaxConfig();

    m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    m_pid = new PIDController(0.8, 0.1, 0.0001);//0.25
    m_pid.setTolerance(0.25);

    m_leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.kElevLimitCurrent);
    m_rightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.kElevLimitCurrent);

    m_rightConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);

    m_rightConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    m_shooterConfig.smartCurrentLimit(ElevatorConstants.kShooterLimitCurrent);

    m_leftMotor.configure(m_leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(m_rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shooterMotor.configure(m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public static SubElev getInstance(){

    if(instance == null){

      instance = new SubElev();
    }
    return instance;
  }

  public void suction(double Power)
  {
    m_shooterMotor.set(Power);
  }

  public void upElev(){

    m_leftMotor.set(0.5);
    m_rightMotor.set(-0.5);
  }

  public void downElev(){

    m_leftMotor.set(-0.5);
    m_rightMotor.set(0.5);
  }

  public void stopElev(){

    m_leftMotor.set(0);
    m_rightMotor.set(0);
  }

  public void shoot(double power){

    m_shooterMotor.set(power);
  }

  public void stopShoot(){

    m_shooterMotor.set(0.0);
  }

  public boolean coral(){

    return m_colorSensor.getProximity() >= ElevatorConstants.kProximity;
  }  

  public void enabledPID(boolean enable){

    enabledPID = enable;
  }

  public void setLevel(int level){

    switch(level){

      case 1:
        m_pid.setSetpoint(ElevatorConstants.kLevel1);
      break;

      case 2:
        m_pid.setSetpoint(ElevatorConstants.kLevel2);
      break;

      case 3:
        m_pid.setSetpoint(ElevatorConstants.kLevel3);
      break;

      default:
        enabledPID = false;
      break;
    }
  }

  public boolean atSetPoint(){

    return m_pid.atSetpoint();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Encoder Elev", SubClimber.getInstance().getEncoderElev());
    SmartDashboard.putBoolean("Coral", coral());
    SmartDashboard.putNumber("Proximity Sensor", m_colorSensor.getProximity());

    SmartDashboard.putNumber("Level", Configure.getLevel());
    SmartDashboard.putNumber("Side", Configure.getSide());
    SmartDashboard.putBoolean("AtSetPointElev", atSetPoint());

    output = m_pid.calculate(SubClimber.getInstance().getEncoderElev());

    if(output > ElevatorConstants.maximumPower)
      output = ElevatorConstants.maximumPower;

    if(output < ElevatorConstants.maximumPower * -1)
      output = ElevatorConstants.maximumPower * -1;

    if(enabledPID){

      m_leftMotor.set(output);
      m_rightMotor.set(-output);
    }
    else{

      m_leftMotor.set(0.0);
      m_rightMotor.set(0.0);
    }
  }
}
