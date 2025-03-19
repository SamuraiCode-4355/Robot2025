package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubLeds;

public class ComShootCoral extends Command {

  private Timer crono;

  public ComShootCoral() {

    addRequirements(SubElev.getInstance());
    crono = new Timer();
  }

  @Override
  public void initialize() {

    crono.reset();
    crono.start();
    SubLeds.violet();
  }

  @Override
  public void execute() {

    /*if(Configure.getLevel() == 1){

      SubElev.getInstance().shoot(-0.2);//-0.3
      SubIntake.getInstance().shoot(0.3);
    }
    else{

      SubElev.getInstance().shoot(-0.45);//-0.3
      SubIntake.getInstance().shoot(0.15);
    }*/
    SubElev.getInstance().shoot(-0.55);//-0.3
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
    return crono.get() >= 1;//1.3
  }
}
