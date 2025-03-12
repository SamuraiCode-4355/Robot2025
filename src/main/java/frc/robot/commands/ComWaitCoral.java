package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubElev;

public class ComWaitCoral extends Command {

  public ComWaitCoral() {

    addRequirements(SubElev.getInstance());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return SubElev.getInstance().coral();
  }
}
