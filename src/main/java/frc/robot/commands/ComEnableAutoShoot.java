package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.Configure;
import frc.robot.subsystems.SubLeds;

public class ComEnableAutoShoot extends Command {

  private boolean enabled;
  private Timer crono;

  public ComEnableAutoShoot(boolean Enabled) {

    crono = new Timer();
    this.enabled = Enabled;
  }

  @Override
  public void initialize() {

    if(enabled){

      Configure.setAutoShoot(true);
      SubLeds.green();
    }
    else{

      Configure.setAutoShoot(false);
      SubLeds.red();
    }

    crono.reset();
    crono.start();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {

    crono.stop();
    crono.reset();

    SubLeds.turnOff();
  }

  @Override
  public boolean isFinished() {
    return crono.get() >= 2;
  }
}
