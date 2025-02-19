package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class SubClimber extends SubsystemBase {

  private static SubClimber instance;
  private SparkMax m_climber;
  private SparkMaxConfig m_climberConfig;
  private RelativeEncoder m_EncoderElev;

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
  }

  public static SubClimber getInstance(){

    if(instance == null){

      instance = new SubClimber();
    }
    return instance;
  }

  public void retractClimber(){

    m_climber.set(0.7);
  }

  public void takeOutClimber(){

    m_climber.set(-0.7);
  }

  public void stopClimber(){

    m_climber.set(0.0);
  }

  public double getEncoderElev(){

    return m_EncoderElev.getPosition();
  }

  @Override
  public void periodic() {}
}
