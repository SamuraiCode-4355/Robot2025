package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
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

  private DigitalInput m_photoSensor;
  private PIDController m_pid;
  private boolean enabledPID;
  private double output;
  private double maxOutput;
  private boolean coralShooting;

  public SubElev() {
    
    m_leftMotor = new SparkMax(ElevatorConstants.kLeftID, MotorType.kBrushless);
    m_rightMotor = new SparkMax(ElevatorConstants.kRightID, MotorType.kBrushless);
    m_shooterMotor = new SparkMax(ElevatorConstants.kShooterID, MotorType.kBrushless);

    m_leftConfig = new SparkMaxConfig();
    m_rightConfig = new SparkMaxConfig();
    m_shooterConfig = new SparkMaxConfig();

    m_photoSensor = new DigitalInput(0);
    
    m_pid = new PIDController(0.5, 0.01, 0.001);//0.25
    m_pid.setTolerance(0.25);

    m_leftConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ElevatorConstants.kElevLimitCurrent);
    m_rightConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ElevatorConstants.kElevLimitCurrent);

    m_rightConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);

    m_rightConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    m_shooterConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ElevatorConstants.kShooterLimitCurrent);

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

    return m_photoSensor.get();//m_colorSensor.getProximity() >= ElevatorConstants.kProximity;
  }  

  public void enabledPID(boolean enable){

    enabledPID = enable;
  }

  public void setLevel(int level){

    switch(level){

      case 1:
        m_pid.setSetpoint(ElevatorConstants.kLevel1);
        maxOutput = ElevatorConstants.maximumPower - 0.2;
        m_pid.setTolerance(1);
      break;

      case 2:
        m_pid.setSetpoint(ElevatorConstants.kLevel2);
        maxOutput = ElevatorConstants.maximumPower;
        m_pid.setTolerance(0.25);
      break;

      case 3:
        m_pid.setSetpoint(ElevatorConstants.kLevel3);
        maxOutput = ElevatorConstants.maximumPower;
        m_pid.setTolerance(0.25);
      break;

      default:
        enabledPID = false;
      break;
    }
  }

  public boolean atSetPoint(){

    return m_pid.atSetpoint();
  }

  public void setIdleMode(boolean Break){

    m_leftConfig.idleMode(Break ? IdleMode.kBrake : IdleMode.kCoast);
    m_rightConfig.idleMode(Break ? IdleMode.kBrake : IdleMode.kCoast);

    m_leftMotor.configure(m_leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(m_rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /*public int getDistance(){

    byte[] buffer = new byte[2];

    distanceSensor.read(SENSOR_ADDRESS, 2, buffer);
    int distance = ((buffer[0] & 0xFF) << 8) | (buffer[1] & 0xFF);
    return distance;
  }*/

  public void isCoralShooting(boolean shooting){

    coralShooting = shooting;
  }

  public boolean coralShooting(){

    return coralShooting;
  }

  public boolean isUpElev(){

    return SubClimber.getInstance().getEncoderElev() >= 1.5;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Encoder Elev", SubClimber.getInstance().getEncoderElev());
    SmartDashboard.putBoolean("Coral", coral());
    SmartDashboard.putNumber("Level", Configure.getLevel());
    SmartDashboard.putNumber("Side", Configure.getSide());
    SmartDashboard.putBoolean("AtSetPointElev", atSetPoint());

    output = m_pid.calculate(SubClimber.getInstance().getEncoderElev());

    if(output > maxOutput)
      output = maxOutput;

    if(output < maxOutput * -1)
      output = maxOutput * -1;

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
