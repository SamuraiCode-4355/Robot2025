package frc.robot;

import frc.robot.commands.ComAccommodation;
import frc.robot.commands.ComLedBlue;
import frc.robot.commands.ComLedwhite;
import frc.robot.commands.ComSwerve;
import frc.robot.commands.ComTurn180;
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

  private final CommandXboxController ControlDrive = new CommandXboxController(0);
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {

    configureBindings();
    
    new Thread(() -> {
      try {
          Thread.sleep(1000);
          autoChooser = AutoBuilder.buildAutoChooser();
          NamedCommands.registerCommand("Dejar Coral", new ComLedBlue());
          NamedCommands.registerCommand("Agarrar Coral", new ComLedwhite());
          SmartDashboard.putData("Auto Chooser", autoChooser);
      } catch (Exception e){}
    }).start();
  }

  private void configureBindings() {

    new Trigger(()-> Math.abs(ControlDrive.getLeftY()) > 0.05 || Math.abs(ControlDrive.getLeftX()) > 0.05 ||
    Math.abs(ControlDrive.getRightX()) > 0.05).whileTrue(new ComSwerve(() -> ControlDrive.getLeftX(),
                                                                       () -> ControlDrive.getLeftY(),
                                                                       () -> ControlDrive.getRightX()));

    ControlDrive.x().whileTrue(new InstantCommand(() -> SubSwerve.getInstance().resetGyro()));
    ControlDrive.leftBumper().whileTrue(new ComAccommodation("LEFT"));
    ControlDrive.rightBumper().whileTrue(new ComAccommodation("RIGHT"));
    ControlDrive.y().onTrue(new ComTurn180());
  }
  

  public Command getAutonomousCommand() {

    return new SequentialCommandGroup(autoChooser.getSelected(), 
                                      new InstantCommand(() -> SubSwerve.getInstance().stop()));
  }
}
