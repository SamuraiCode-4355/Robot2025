package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.Configure;
import frc.robot.subsystems.SubElev;

public class ComUpElev extends Command {

  private boolean finish;
  private int config;

  public ComUpElev(){

    addRequirements(SubElev.getInstance());
  }

  public ComUpElev(int Config) {

    addRequirements(SubElev.getInstance());
    this.config = Config;
  }

  @Override
  public void initialize() {

    if(SubElev.getInstance().coral())
      finish = true;

    SubElev.getInstance().setIdleMode(true);
    SubElev.getInstance().setLevel((config == 0) ? Configure.getLevel() : config);
    SubElev.getInstance().enabledPID(true);
  }

  @Override
  public void execute() {

   /*  if(SubElev.getInstance().atSetPoint())
      finish = true;*/
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
