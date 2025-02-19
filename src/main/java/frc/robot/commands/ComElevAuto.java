package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.Configure;
import frc.robot.subsystems.SubElev;

public class ComElevAuto extends Command {

  private Timer crono;
  private boolean shoot;

  public ComElevAuto() {

    addRequirements(SubElev.getInstance());
    crono = new Timer();
  }

  @Override
  public void initialize() {

    SubElev.getInstance().setLevel(Configure.getLevel());
    SubElev.getInstance().enabledPID(true);
    crono.reset();
  }

  @Override
  public void execute() {

    if(SubElev.getInstance().atSetPoint() && !shoot){
      
      SubElev.getInstance().shoot(-0.3);
      crono.start();
      shoot = true;
    }
  }

  @Override
  public void end(boolean interrupted) {

    SubElev.getInstance().enabledPID(false);
    SubElev.getInstance().stopElev();
    SubElev.getInstance().stopShoot();
    crono.stop();
    crono.reset();
    shoot = false;

    new ComDownElev().schedule();
  }

  @Override
  public boolean isFinished() {
    return crono.get() >= 0.7;
  }
}
