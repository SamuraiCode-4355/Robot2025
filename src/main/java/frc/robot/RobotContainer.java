package frc.robot;

import frc.robot.commands.ComArrangement;
import frc.robot.commands.ComClimber;
import frc.robot.commands.ComElev;
import frc.robot.commands.ComElevAuto;
import frc.robot.commands.ComIntakeJoint;
import frc.robot.commands.ComIntakeSuction;
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
  private final CommandXboxController m_TestControl = new CommandXboxController(2);
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

    NamedCommands.registerCommand("Left Reef", new ComArrangement(1));
    NamedCommands.registerCommand("Right Reef", new ComArrangement(2));
  }

  private void configureBindings() {

    new Trigger(()-> Math.abs(m_DriveControl.getLeftY()) > 0.05 || Math.abs(m_DriveControl.getLeftX()) > 0.05 ||
    Math.abs(m_DriveControl.getRightX()) > 0.05).whileTrue(new ComSwerve(() -> m_DriveControl.getLeftX(),
                                                                         () -> m_DriveControl.getLeftY(),
                                                                         () -> m_DriveControl.getRightX()));

    m_DriveControl.x().whileTrue(new InstantCommand(() -> SubSwerve.getInstance().resetGyro()));
    m_DriveControl.b().onTrue(new ComArrangement(0));
    m_DriveControl.y().whileTrue(new InstantCommand(() -> SubSwerve.getInstance().increaseCurrentTurn()));
    m_DriveControl.a().whileTrue(new InstantCommand(() -> SubSwerve.getInstance().decreaseCurrentTurn()));

    m_DriveControl.pov(0).whileTrue(new InstantCommand(() -> SubSwerve.getInstance().increaseCurrentDrive()));
    m_DriveControl.pov(180).whileTrue(new InstantCommand(() -> SubSwerve.getInstance().decreaseCurrentDrive()));

    m_DriveControl.pov(270).whileTrue(new InstantCommand(() -> SubSwerve.getInstance().decreaseCurrentTurn()));
    m_DriveControl.pov(90).whileTrue(new InstantCommand(() -> SubSwerve.getInstance().increaseCurrentTurn()));

   // ControlDrive.y().onTrue(new ComTurn180());
   /*  ControlMeca.pov(270).whileTrue(new InstantCommand(() -> Configure.setSide(1)));
    ControlMeca.pov(90).whileTrue(new InstantCommand(() -> Configure.setSide(2)));
    ControlMeca.a().whileTrue(new InstantCommand(() -> Configure.setLevel(1)));
    ControlMeca.b().whileTrue(new InstantCommand(() -> Configure.setLevel(2)));
    ControlMeca.y().whileTrue(new InstantCommand(() -> Configure.setLevel(3)));*/

    new Trigger(() -> m_DriveControl.getLeftTriggerAxis() > 0.1).whileTrue(new ComElev(() -> m_DriveControl.getLeftTriggerAxis()));
    new Trigger(() -> m_DriveControl.getRightTriggerAxis() > 0.1).whileTrue(new ComElev(() -> m_DriveControl.getLeftTriggerAxis()));
  
    m_MechaControl.leftBumper().whileTrue(new ComClimber(false));
    m_MechaControl.rightBumper().whileTrue(new ComClimber(true));

    m_MechaControl.a().onTrue(new ComTakeCoral());
    m_MechaControl.y().whileTrue(new ComShootCoral());
    //m_DriveControl.b().whileTrue(new SuctionCoral());
    

    m_MechaControl.pov(270).whileTrue(new ComIntakeJoint(true));
    m_MechaControl.pov(90).whileTrue(new ComIntakeJoint(false));

    m_MechaControl.x().whileTrue(new ComIntakeSuction(true));
    m_MechaControl.b().whileTrue(new ComIntakeSuction(false));

    m_TestControl.leftBumper().onTrue(new ComElevAuto());

    m_TestControl.pov(270).whileTrue(new InstantCommand(() -> Configure.setSide(1)));
    m_TestControl.pov(90).whileTrue(new InstantCommand(() -> Configure.setSide(2)));

    m_TestControl.a().whileTrue(new InstantCommand(() -> Configure.setLevel(1)));
    m_TestControl.b().whileTrue(new InstantCommand(() -> Configure.setLevel(2)));
    m_TestControl.y().whileTrue(new InstantCommand(() -> Configure.setLevel(3)));
    m_TestControl.x().whileTrue(new ComShootCoral());
  }

  public Command getAutonomousCommand() {

    return new SequentialCommandGroup(m_autoChooser.getSelected(), 
                                      new InstantCommand(() -> SubSwerve.getInstance().stop()));
  }
}
