package frc.robot;

import frc.robot.commands.ComLedBlue;
import frc.robot.commands.ComLedwhite;
import frc.robot.commands.ComSwerve;
import frc.robot.subsystems.SubSwerve;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final CommandXboxController ControlDrive = new CommandXboxController(0);

  public RobotContainer() {

    configureBindings();
    NamedCommands.registerCommand("Dejar Coral", new ComLedBlue());
    NamedCommands.registerCommand("Agarrar Coral", new ComLedwhite());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    new Trigger(()-> Math.abs(ControlDrive.getLeftY()) > 0.05 || Math.abs(ControlDrive.getLeftX()) > 0.05 ||
    Math.abs(ControlDrive.getRightX()) > 0.05).whileTrue(new ComSwerve(() -> ControlDrive.getLeftX(),
                                                                       () -> ControlDrive.getLeftY(),
                                                                       () -> ControlDrive.getRightX()));
   // ControlDrive.a().whileTrue(new ComPrueba());

    ControlDrive.x().whileTrue(new InstantCommand(() -> SubSwerve.getInstance().resetGyro()));
    /*ControlDrive.leftBumper().whileTrue(new ComContinue());
    ControlDrive.rightBumper().whileTrue(new ComContinueR());*/
  }


  public Command getAutonomousCommand() {

    return new SequentialCommandGroup(new PathPlannerAuto("AutoC1"),
                                      new InstantCommand(() -> SubSwerve.getInstance().stop()));
  }
}
