package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.math.Configure;

public class SubShooter extends SubsystemBase {

  private VictorSPX mShooter;
  private static SubShooter instance;

  public SubShooter() {

    mShooter = new VictorSPX(ShooterConstants.kShooterID);
  }

  public static SubShooter getInstance(){

    if(instance == null){

      instance = new SubShooter();
    }
    return instance;
  }

  public void shoot(){

    mShooter.set(ControlMode.PercentOutput, -0.65);
  }

  public void suction(){

    mShooter.set(ControlMode.PercentOutput, 0.5);
  }

  public void stop(){

    mShooter.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Level", Configure.getLevel());
    SmartDashboard.putNumber("Side", Configure.getSide());
  }
}
