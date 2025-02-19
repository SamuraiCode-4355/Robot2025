package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubElev;

public class ComDownElev extends Command {

  private boolean finish;

  public ComDownElev() {

    addRequirements(SubElev.getInstance());
  }

  @Override
  public void initialize() {

    SubElev.getInstance().setLevel(1);
    SubElev.getInstance().enabledPID(true);
  }

  @Override
  public void execute() {

    if(SubElev.getInstance().atSetPoint())
      finish = true;
  }

  @Override
  public void end(boolean interrupted) {

    SubElev.getInstance().enabledPID(false);
    SubElev.getInstance().stopShoot();
    finish = false;
  }

  @Override
  public boolean isFinished() {
    return finish;
  }
}
