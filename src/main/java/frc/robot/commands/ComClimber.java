package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubClimber;

public class ComClimber extends Command {

  private boolean rightBumper;

  public ComClimber(boolean RightBumper) {

    addRequirements(SubClimber.getInstance());
    this.rightBumper = RightBumper;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if(rightBumper){

      SubClimber.getInstance().retractClimber();
    }
    else{

      SubClimber.getInstance().takeOutClimber();
    }
  }

  @Override
  public void end(boolean interrupted) {

    SubClimber.getInstance().stopClimber();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
