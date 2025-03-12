package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.math.Configure;
import frc.robot.subsystems.SubLeds;
import frc.robot.subsystems.SubSwerve;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {

    m_robotContainer = new RobotContainer();
    SubLeds.turnOff();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    SubLeds.turnOff();
    SubSwerve.getInstance().setBreak(true);
    SubSwerve.getInstance().initEncoders();
    Configure.setAutonomo(true);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Configure.setAutonomo(false);
    SubLeds.violet();
    SubSwerve.getInstance().initEncoders();
    SubSwerve.getInstance().setBreak(false);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
