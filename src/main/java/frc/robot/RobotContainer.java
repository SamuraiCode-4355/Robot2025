package frc.robot;

import frc.robot.commands.ComClimber;
import frc.robot.commands.ComCoralStation;
import frc.robot.commands.ComDownElev;
import frc.robot.commands.ComUpElev;
import frc.robot.commands.ComIntake;
import frc.robot.commands.ComShootCoral;
import frc.robot.commands.ComSwerve;
import frc.robot.commands.ComTakeCoral;
import frc.robot.math.Configure;
import frc.robot.subsystems.SubElev;
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

  private SendableChooser<Command> m_autoChooser;

  public RobotContainer() {

    configureBindings();
    
    new Thread(() -> {
      try {
          Thread.sleep(1000);
          m_autoChooser = AutoBuilder.buildAutoChooser();
          NamedCommands.registerCommand("Shoot", new ComShootCoral());
          NamedCommands.registerCommand("Level2", new ComUpElev(2));
          NamedCommands.registerCommand("DownElev", new ComDownElev());
          NamedCommands.registerCommand("CoralStation", new ComCoralStation());
          SmartDashboard.putData("Auto Chooser", m_autoChooser);
      } catch (Exception e){}
    }).start();
  }

  private void configureBindings() {

    new Trigger(() -> SubElev.getInstance().coral()).onTrue(new ComTakeCoral());

    new Trigger(()-> Math.abs(m_DriveControl.getLeftY()) > 0.08 || Math.abs(m_DriveControl.getLeftX()) > 0.08 ||
    Math.abs(m_DriveControl.getRightX()) > 0.08).whileTrue(new ComSwerve(() -> m_DriveControl.getLeftX(),
                                                                         () -> m_DriveControl.getLeftY(),
                                                                         () -> m_DriveControl.getRightX()));

    m_DriveControl.x().whileTrue(new InstantCommand(() -> SubSwerve.getInstance().resetGyro()));
    m_DriveControl.leftBumper().whileTrue(new ComIntake(false));
    m_DriveControl.rightBumper().whileTrue(new ComIntake(true));

    new Trigger(() -> m_DriveControl.getLeftTriggerAxis() >= 0.1).whileTrue(new ComShootCoral());
    new Trigger(() -> m_DriveControl.getRightTriggerAxis() >= 0.1).onTrue(new ComUpElev(0));

    m_MechaControl.pov(270).whileTrue(new ComClimber(false));
    m_MechaControl.pov(90).whileTrue(new ComClimber(true));

    m_MechaControl.pov(180).onTrue(new ComDownElev());

    new Trigger(() -> m_MechaControl.getRightTriggerAxis() > 0.1).onTrue(new ComTakeCoral());

    m_MechaControl.a().whileTrue(new InstantCommand(() -> Configure.setLevel(1)));
    m_MechaControl.b().whileTrue(new InstantCommand(() -> Configure.setLevel(2)));
    m_MechaControl.y().whileTrue(new InstantCommand(() -> Configure.setLevel(3)));
  }
/* 
  public static boolean joystickActive(){

    return Math.abs(m_DriveControl.getLeftY()) > 0.05 || Math.abs(m_DriveControl.getLeftX()) > 0.05 ||
    Math.abs(m_DriveControl.getRightX()) > 0.05;
  }
*/
  public Command getAutonomousCommand() {

    return new SequentialCommandGroup(m_autoChooser.getSelected(), 
                                      new InstantCommand(() -> SubSwerve.getInstance().stop()));
  }
}
