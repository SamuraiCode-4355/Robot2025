package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConst;

public class SubLeds extends SubsystemBase {

  private static SubLeds instance;
  private Spark m_Led;

  public SubLeds() {

    m_Led = new Spark(RobotConst.kLedPort);
  }

  public static SubLeds getInstance(){

    if(instance == null){

      instance = new SubLeds();
    }
    return instance;
  }

  public void ledOff(){

    m_Led.set(0.99);
  }

  public void ledBlue(){

    m_Led.set(-0.29);
  }

  public void ledWhite(){

    m_Led.set(-0.21);
  }

  @Override
  public void periodic() {}
}
