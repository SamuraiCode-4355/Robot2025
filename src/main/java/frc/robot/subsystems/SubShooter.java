package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.math.Configure;

public class SubShooter extends SubsystemBase {

  private SparkMax mShooter;
  private static SubShooter instance;

  public SubShooter() {

    mShooter = new SparkMax(ShooterConstants.kShooterID, MotorType.kBrushed);
  }

  public static SubShooter getInstance(){

    if(instance == null){

      instance = new SubShooter();
    }
    return instance;
  }

  public void shoot(){

    mShooter.set(0.65);
  }

  public void suction(){

    mShooter.set(-0.5);
  }

  public void stop(){

    mShooter.set(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Level", Configure.getLevel());
    SmartDashboard.putNumber("Side",Configure.getSide());
    SmartDashboard.putNumber("Side Reef", Configure.getSideReef());
  }
}
