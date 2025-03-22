package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubLeds;

public class ComShootCoral extends Command {

  private Timer crono;
  private boolean autonomo;

  public ComShootCoral(boolean Autonomo) {

    addRequirements(SubElev.getInstance());
    crono = new Timer();
    this.autonomo = Autonomo;
  }

  @Override
  public void initialize() {

    crono.reset();
    crono.start();
    SubLeds.violet();
  }

  @Override
  public void execute() {

    if(autonomo){

      SubElev.getInstance().shoot(-0.1);
    }
    else{

      SubElev.getInstance().shoot(-0.28);
    }
  }

  @Override
  public void end(boolean interrupted) {

    SubElev.getInstance().stopShoot();
    SubLeds.turnOff();

    crono.stop();
    crono.reset();
  }

  @Override
  public boolean isFinished() {
    return crono.get() >= 1;
  }
}
