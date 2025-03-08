package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class SubLeds extends SubsystemBase {

  private static Spark m_Led = new Spark(RobotConstants.kLedPort);

  public static void turnOff(){

    m_Led.set(0.99);
  }

  public static void lightChaseBlue(){

    m_Led.set(-0.29);
  }

  public static void strobeBlue(){

    m_Led.set(-0.29);
  }

  public static void heartbeatBlue(){

    m_Led.set(-0.23);
  }

  public static void shotBlue(){

    m_Led.set(-0.83);
  }

  public static void violet(){

    m_Led.set(0.91);
  }

  public static void blueViolet(){

    m_Led.set(0.89);
  }

  @Override
  public void periodic() {}
}
