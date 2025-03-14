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

    /*if(SubElev.getInstance().coral())
      finish = true;*/

    SubElev.getInstance().setIdleMode(true);
    SubElev.getInstance().setLevel(level);
  }

  @Override
  public void execute() {

    if(!SubElev.getInstance().coral())
      SubElev.getInstance().enabledPID(true);

    /*if(Configure.getAutoShoot() && level != 1){

      if(SubElev.getInstance().atSetPoint() && SubIntake.getInstance().reefArrangement()){

        finish = true;
      }
    }
    else{*/

    if(SubElev.getInstance().atSetPoint()){

      finish = true;
    }
  }

  @Override
  public void end(boolean interrupted) {

    SubElev.getInstance().enabledPID(false);
    SubElev.getInstance().stopShoot();
    finish = false;

    /*if(SubIntake.getInstance().reefArrangement() && Configure.getAutoShoot() && level != 1)
      new ComShootCoral().schedule();*/
  }

  @Override
  public boolean isFinished() {
    return finish;
  }
}
