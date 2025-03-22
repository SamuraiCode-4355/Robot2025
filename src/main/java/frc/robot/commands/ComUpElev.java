package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubIntake;

public class ComUpElev extends Command {

  private boolean finish;
  private int level;

  public ComUpElev(int Level) {

    addRequirements(SubElev.getInstance(), SubIntake.getInstance());
    this.level = Level;
  }

  @Override
  public void initialize() {

    SubElev.getInstance().setLevel(level);
  }

  @Override
  public void execute() {

    if(!SubElev.getInstance().coral())
      SubElev.getInstance().enabledPID(true);

    if(SubElev.getInstance().atSetPoint())
      finish = true;
  }

  @Override
  public void end(boolean interrupted) {

    finish = false;
  }

  @Override
  public boolean isFinished() {
    return finish;
  }
}
