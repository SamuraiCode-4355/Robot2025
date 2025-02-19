package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubIntake;

public class ComIntakeJoint extends Command {

  private boolean pov270;

  public ComIntakeJoint(boolean Pov270) {

    addRequirements(SubIntake.getInstance());

    this.pov270 = Pov270;
  }

  @Override
  public void initialize() {

    if(pov270){

      SubIntake.getInstance().downIntake();
    }
    else{

      SubIntake.getInstance().upIntake();
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {

    SubIntake.getInstance().stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
