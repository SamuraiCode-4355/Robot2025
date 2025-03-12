package frc.robot;

import frc.robot.commands.ComClimber;
import frc.robot.commands.ComDownElev;
import frc.robot.commands.ComUpElev;
import frc.robot.commands.ComWaitCoral;
import frc.robot.math.Configure;
import frc.robot.commands.ComIntake;
import frc.robot.commands.ComSeaweed;
import frc.robot.commands.ComShootCoral;
import frc.robot.commands.ComSwerve;
import frc.robot.commands.ComTakeCoral;
import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubLeds;
import frc.robot.subsystems.SubSwerve;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final CommandXboxController m_DriveControl = new CommandXboxController(0);
  private final CommandXboxController m_MechaControl = new CommandXboxController(1);
  private final CommandXboxController m_testControl = new CommandXboxController(2);

  private SendableChooser<Command> m_autoChooser;

  public RobotContainer() {

    configureBindings();
    
    new Thread(() -> {
      try {
          Thread.sleep(1000);
          m_autoChooser = AutoBuilder.buildAutoChooser();
          SmartDashboard.putData("Auto Chooser", m_autoChooser);
      } catch (Exception e){}
    }).start();

    NamedCommands.registerCommand("Shoot", new ComShootCoral());
    NamedCommands.registerCommand("Level2", new ComUpElev(2));
    NamedCommands.registerCommand("DownElev", new ComDownElev());
    NamedCommands.registerCommand("TakeCoral", new ComTakeCoral());
    NamedCommands.registerCommand("WaitCoral", new ComWaitCoral());
  }

  private void configureBindings() {

    new Trigger(() -> SubElev.getInstance().coral() && !Configure.getAutonomo()).onTrue(new ComTakeCoral());

    new Trigger(() -> Math.abs(m_DriveControl.getLeftY()) > 0.08 || Math.abs(m_DriveControl.getLeftX()) > 0.08 ||
    Math.abs(m_DriveControl.getRightX()) > 0.08).whileTrue(new ComSwerve(() -> m_DriveControl.getLeftX(),
                                                                         () -> m_DriveControl.getLeftY(),
                                                                         () -> m_DriveControl.getRightX()));

    m_DriveControl.x().whileTrue(new InstantCommand(() -> SubSwerve.getInstance().resetGyro()));
    //m_DriveControl.leftBumper().whileTrue(new ComIntake(false));
    //m_DriveControl.rightBumper().whileTrue(new ComIntake(true));

    new Trigger(() -> m_DriveControl.getLeftTriggerAxis() >= 0.1).whileTrue(new ComShootCoral());
    //new Trigger(() -> m_DriveControl.getRightTriggerAxis() >= 0.1).onTrue(new ComArrangement());

    m_MechaControl.leftBumper().whileTrue(new ComClimber(false));
    m_MechaControl.rightBumper().whileTrue(new ComClimber(true));

    new Trigger(() -> m_MechaControl.getLeftTriggerAxis() >= 0.1).whileTrue(new ComIntake(false));
    new Trigger(() -> m_MechaControl.getRightTriggerAxis() >= 0.1).whileTrue(new ComIntake(true));

    m_MechaControl.pov(0).onTrue(new ComUpElev(0));
    m_MechaControl.pov(180).onTrue(new ComDownElev());

    //m_MechaControl.pov(270).whileTrue(new InstantCommand(() -> Configure.setDrive(true)));
    //m_MechaControl.pov(90).whileTrue(new InstantCommand(() -> Configure.setDrive(false)));
    m_MechaControl.pov(270).whileTrue(new InstantCommand(() -> Configure.setAutoShoot(false)));
    m_MechaControl.pov(90).whileTrue(new InstantCommand(() -> Configure.setAutoShoot(true)));

    m_MechaControl.a().whileTrue(new InstantCommand(() -> Configure.setLevel(1)));
    m_MechaControl.b().whileTrue(new InstantCommand(() -> Configure.setLevel(2)));
    m_MechaControl.y().whileTrue(new InstantCommand(() -> Configure.setLevel(3)));
    m_MechaControl.x().onTrue(new ComSeaweed(() -> m_MechaControl.x().getAsBoolean()));

    m_testControl.a().whileTrue(new InstantCommand(() -> SubLeds.heartbeatBlue()));
    m_testControl.b().whileTrue(new InstantCommand(() -> SubLeds.lightChaseBlue()));
    m_testControl.x().whileTrue(new InstantCommand(() -> SubLeds.shotBlue()));
    m_testControl.y().whileTrue(new InstantCommand(() -> SubLeds.strobeBlue()));

    m_testControl.pov(0).whileTrue(new InstantCommand(() -> SubLeds.turnOff()));
    m_testControl.pov(90).whileTrue(new InstantCommand(() -> SubLeds.blueViolet()));
    m_testControl.pov(270).whileTrue(new InstantCommand(() -> SubLeds.violet()));
  }

  public Command getAutonomousCommand() {

    return new SequentialCommandGroup(m_autoChooser.getSelected(), 
                                      new InstantCommand(() -> SubSwerve.getInstance().stop()));
  }
}
