package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.Configure;
import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubIntake;

public class ComUpElev extends Command {

  private Timer crono;
  private boolean shoot;
  private boolean finish;
  private int config;

  public ComUpElev(int Config) {

    addRequirements(SubElev.getInstance(), SubIntake.getInstance());
    crono = new Timer();
    this.config = Config;
  }

  @Override
  public void initialize() {

    if(SubElev.getInstance().coral())
      finish = true;

    SubElev.getInstance().setIdleMode(true);
    SubElev.getInstance().setLevel((config == 0) ? Configure.getLevel() : config);
    SubElev.getInstance().enabledPID(true);
    crono.reset();
  }

  @Override
  public void execute() {

    if(SubElev.getInstance().atSetPoint()){
      
      if(SubIntake.getInstance().reefArrangement() && !shoot){

        crono.start();
        SubElev.getInstance().shoot(-0.45);//-0.3
        SubIntake.getInstance().shoot(0.15);  
        shoot = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {

    SubElev.getInstance().enabledPID(false);
    SubElev.getInstance().stopElev();
    SubElev.getInstance().stopShoot();
    SubIntake.getInstance().stop();

    if(!SubElev.getInstance().coral())
      SubElev.getInstance().isCoralShooting(true);

    crono.stop();
    crono.reset();
    shoot = false;
    finish = false;
   // new ComDownElev().schedule();
  }

  @Override
  public boolean isFinished() {
    return crono.get() >= 0.7 || finish;
  }
}
