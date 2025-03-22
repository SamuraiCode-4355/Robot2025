package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.math.Configure;

public class SubClimber extends SubsystemBase {

  private static SubClimber instance;

  private SparkMax m_climber;
  private SparkMaxConfig m_climberConfig;
  private RelativeEncoder m_EncoderElev;
  private Rev2mDistanceSensor climberSensor;

  //-------------------MÉTODO CONSTRUCTOR--------------------------

  public SubClimber() {

    m_climber = new SparkMax(ClimberConstants.kClimberID, MotorType.kBrushed);
    m_climberConfig = new SparkMaxConfig();

    m_EncoderElev = m_climber.getEncoder();

    m_climberConfig.idleMode(IdleMode.kBrake);

    m_climberConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

    m_climberConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    m_climber.configure(m_climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
    climberSensor.setAutomaticMode(true);
  }

  //--------------------MÉTODO DE FABRICA-------------------------

  public static SubClimber getInstance(){

    if(instance == null){

      instance = new SubClimber();
    }
    return instance;
  }

  //----------------------ATRIBUTOS-------------------------------

  public double getEncoderElev(){

    return m_EncoderElev.getPosition();
  }

  public boolean isElevUp(){

    return m_EncoderElev.getPosition() >= ElevatorConstants.kLevel2 - 1.5;
  }

  public boolean robotUp(){

    return climberSensor.getRange() > 0 && climberSensor.getRange() <= ClimberConstants.kDistanceSensor;
  }

  //---------------------MÉTODOS----------------------------------

  public void retractClimber(){

    m_climber.set(0.9);
  }

  public void takeOutClimber(){

    m_climber.set(-0.9);
  }

  public void stopClimber(){

    m_climber.set(0.0);
  }

  //------------------MÉTODO PERIODICO---------------------------------

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Level", Configure.getLevel());
    SmartDashboard.putNumber("Side", Configure.getSide());
    SmartDashboard.putNumber("climber sensor", climberSensor.getRange());

    SmartDashboard.putBoolean("RobotUp", robotUp());
    SmartDashboard.putBoolean("AutoShoot", Configure.getAutoShoot());
    SmartDashboard.putBoolean("Drive", Configure.getDrive());
    SmartDashboard.putBoolean("Autonomo", Configure.getAutonomo());
  }
}
