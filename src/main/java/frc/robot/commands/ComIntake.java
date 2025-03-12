package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubIntake;

public class ComIntake extends Command {

  private boolean shoot;

  public ComIntake(boolean shoot) {

    addRequirements(SubIntake.getInstance());
    this.shoot = shoot;
  }

  @Override
  public void initialize() {

    if(shoot){

      SubIntake.getInstance().setBreak(false);
      SubIntake.getInstance().shoot(0.6);
    }
    else{

      SubIntake.getInstance().suction();
      SubIntake.getInstance().setBreak(true);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {

    SubIntake.getInstance().stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
