package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.Configure;
import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubLeds;

public class ComTakeCoral extends Command {

  private Timer crono;
  private int config;

  public ComTakeCoral() {

    addRequirements(SubElev.getInstance());
    crono = new Timer();
  }

  public ComTakeCoral(int Config){

    addRequirements(SubElev.getInstance());
    crono = new Timer();
    this.config = Config;
  }

  @Override
  public void initialize() {

    SubElev.getInstance().suction();
    //Prender leds azul
    crono.reset();
    crono.start();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {

    SubElev.getInstance().stopShoot();
    SubLeds.turnOff();

    if(crono.get() >= 0.8)
      Configure.setCoral(true);

    if(config != 0)
      new ComUpElev(config).schedule();

    crono.stop();
    crono.reset();
  }

  @Override
  public boolean isFinished() {
    return !SubElev.getInstance().coral();
  }
}