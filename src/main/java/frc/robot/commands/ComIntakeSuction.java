package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubIntake;

public class ComIntakeSuction extends Command {

  private boolean x_Botton;

  public ComIntakeSuction(boolean X_Botton) {

    addRequirements(SubIntake.getInstance());

    this.x_Botton = X_Botton;
  }

  @Override
  public void initialize() {

    if(x_Botton){

      SubIntake.getInstance().suction();
    }
    else{

      SubIntake.getInstance().shoot();
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {

    SubIntake.getInstance().stopSuction();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
