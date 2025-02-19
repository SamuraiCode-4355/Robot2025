package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class SubIntake extends SubsystemBase {

  private static SubIntake instance;
  
  private SparkMax m_jointMotor;
  private SparkMax m_suctionMotor;

  private SparkMaxConfig m_jointConfig;
  private SparkMaxConfig m_suctionConfig;

  public SubIntake() {

    m_jointMotor = new SparkMax(IntakeConstants.kJointID, MotorType.kBrushless);
    m_suctionMotor = new SparkMax(IntakeConstants.kSuctionID, MotorType.kBrushless);

    m_jointConfig = new SparkMaxConfig();
    m_suctionConfig = new SparkMaxConfig();

    m_jointConfig.smartCurrentLimit(IntakeConstants.kJointLimitCurrent);
    m_suctionConfig.smartCurrentLimit(IntakeConstants.kSuctionLimitCurrent);
  }

  public static SubIntake getInstance(){

    if(instance == null){

      instance = new SubIntake();
    }
    return instance;
  }

  public void downIntake(){

    m_jointMotor.set(0.3);
  }

  public void upIntake(){

    m_jointMotor.set(-0.3);
  }

  public void stopIntake(){

    m_jointMotor.set(0.0);
  }

  public void suction(){

    m_suctionMotor.set(0.6);
  }

  public void shoot(){

    m_suctionMotor.set(-1);//-0.7
  }

  public void stopSuction(){

    m_suctionMotor.set(0.0);
  }

  public double getJointPosition(){

    return m_jointMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Encoder Position", getJointPosition());
  }
}
