package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubElev;

public class ComTakeCoral extends Command {

  private boolean coral;

  public ComTakeCoral() {

    addRequirements(SubElev.getInstance());
  }

  @Override
  public void initialize() {

   // SubElev.getInstance().shoot(0.2);
  }

  @Override
  public void execute() {

    if(SubElev.getInstance().coral())
      coral = true;

    if(coral){

      SubElev.getInstance().shoot(-0.09);//-0.1
    }
  }

  @Override
  public void end(boolean interrupted) {

    SubElev.getInstance().stopShoot();
    coral = false;
    SubElev.getInstance().isCoralShooting(false);
  }

  @Override
  public boolean isFinished() {

    return coral && !SubElev.getInstance().coral();
  }
}
