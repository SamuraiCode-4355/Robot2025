package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class SubIntake extends SubsystemBase {

  private static SubIntake instance;

  private SparkMax mIntakeMotor;
  private SparkMaxConfig mIntakeConfig;
  private Rev2mDistanceSensor mDistanceSensor;

  //------------------MÉTODO CONSTRUCTOR---------------------------

  public SubIntake() {

    mIntakeMotor = new SparkMax(IntakeConstants.kIntakeID, MotorType.kBrushless);
    mIntakeConfig = new SparkMaxConfig();

    mIntakeConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(IntakeConstants.kIntakeLimitCurrent);
    mIntakeMotor.configure(mIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mDistanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
    mDistanceSensor.setAutomaticMode(true);
  }

  //------------------------METODO DE FÁBRICA--------------------------

  public static SubIntake getInstance(){

    if(instance == null){

      instance = new SubIntake();
    }
    return instance;
  }

  //--------------------------ATRIBUTOS----------------------------------

  public boolean reefArrangement(){

    return mDistanceSensor.getRange() >= IntakeConstants.kMinReefSensor 
        && mDistanceSensor.GetRange() <= IntakeConstants.kMaxReefSensor;
  }

  //---------------------------MÉTODOS------------------------------------

  public void shoot(double speed){

    mIntakeMotor.set(speed);
  }

  public void suction(){

    mIntakeMotor.set(-0.6);
  }

  public void stop(){

    mIntakeMotor.set(0.0);
  }

  public void setBreak(boolean Break){

    mIntakeConfig.idleMode(Break ? IdleMode.kBrake : IdleMode.kCoast);
    mIntakeMotor.configure(mIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //------------------------MÉTODO PERIODICO-------------------------------

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Sensor2m", mDistanceSensor.getRange());
    SmartDashboard.putBoolean("ReefArrangement", reefArrangement());
  }
}
