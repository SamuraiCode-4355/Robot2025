package frc.robot;

import frc.robot.commands.AutoShooter;
import frc.robot.commands.ComArrangement;
import frc.robot.commands.ComShooter;
import frc.robot.commands.ComSwerve;
import frc.robot.math.Configure;
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

  private CommandXboxController ControlDrive = new CommandXboxController(0);
//  private CommandXboxController ControlMeca = new CommandXboxController(1);
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {

    configureBindings();
    
    new Thread(() -> {
      try {
          Thread.sleep(1000);
          autoChooser = AutoBuilder.buildAutoChooser();
          SmartDashboard.putData("Auto Chooser", autoChooser);
      } catch (Exception e){}
    }).start();

    NamedCommands.registerCommand("Shoot", new AutoShooter());
    NamedCommands.registerCommand("Left Reef", new ComArrangement(1));
    NamedCommands.registerCommand("Right Reef", new ComArrangement(2));
  }

  private void configureBindings() {

    new Trigger(()-> Math.abs(ControlDrive.getLeftY()) > 0.05 || Math.abs(ControlDrive.getLeftX()) > 0.05 ||
    Math.abs(ControlDrive.getRightX()) > 0.05).whileTrue(new ComSwerve(() -> ControlDrive.getLeftX(),
                                                                       () -> ControlDrive.getLeftY(),
                                                                       () -> ControlDrive.getRightX()));

    ControlDrive.x().whileTrue(new InstantCommand(() -> SubSwerve.getInstance().resetGyro()));
    ControlDrive.b().onTrue(new ComArrangement(0));

    ControlDrive.y().whileTrue(new InstantCommand(() -> SubSwerve.getInstance().increaseCurrentTurn()));
    ControlDrive.a().whileTrue(new InstantCommand(() -> SubSwerve.getInstance().decreaseCurrentTurn()));

    ControlDrive.pov(0).whileTrue(new InstantCommand(() -> SubSwerve.getInstance().increaseCurrentDrive()));
    ControlDrive.pov(180).whileTrue(new InstantCommand(() -> SubSwerve.getInstance().decreaseCurrentDrive()));
    //ControlDrive.leftBumper().whileTrue(new ComAccommodation());
   // ControlDrive.y().onTrue(new ComTurn180());
    ControlDrive.leftBumper().whileTrue(new ComShooter(() -> ControlDrive.rightBumper().getAsBoolean()));
    ControlDrive.rightBumper().whileTrue(new ComShooter(() -> ControlDrive.rightBumper().getAsBoolean()));

   /*  ControlMeca.pov(270).whileTrue(new InstantCommand(() -> Configure.setSide((byte) 1)));
    ControlMeca.pov(90).whileTrue(new InstantCommand(() -> Configure.setSide((byte) 2)));
    ControlMeca.a().whileTrue(new InstantCommand(() -> Configure.setLevel((byte) 1)));
    ControlMeca.b().whileTrue(new InstantCommand(() -> Configure.setLevel((byte) 2)));
    ControlMeca.y().whileTrue(new InstantCommand(() -> Configure.setLevel((byte) 3)));*/
    ControlDrive.pov(270).whileTrue(new InstantCommand(() -> Configure.setSide((byte) 1)));
    ControlDrive.pov(90).whileTrue(new InstantCommand(() -> Configure.setSide((byte) 2)));
  }

  public Command getAutonomousCommand() {

    return new SequentialCommandGroup(autoChooser.getSelected(), 
                                      new InstantCommand(() -> SubSwerve.getInstance().stop()));
  }
}
