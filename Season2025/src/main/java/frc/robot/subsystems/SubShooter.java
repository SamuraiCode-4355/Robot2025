package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShootConstant;
import frc.robot.math.Configure;

public class SubShooter extends SubsystemBase {

  private SparkMax M1;
  private static SubShooter Instance;

  public SubShooter() {

    M1 = new SparkMax(ShootConstant.KM1, MotorType.kBrushed);
  }

  public static SubShooter GetInstance(){

    if(Instance == null){

      Instance = new SubShooter();
    }
    return Instance;
  }

  public void Shoot(){

    M1.set(0.5);
  }

  public void Stop(){

    M1.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Level", Configure.getLevel());
    SmartDashboard.putNumber("Lado",Configure.getSide());
  }
}
