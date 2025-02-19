package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubElev;

public class ComElev extends Command {

  private DoubleSupplier leftTrigger;

  public ComElev(DoubleSupplier LeftTrigger) {

    this.leftTrigger = LeftTrigger;
    addRequirements(SubElev.getInstance());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if(leftTrigger.getAsDouble() > 0.1){

      SubElev.getInstance().upElev();
    }
    else{

      SubElev.getInstance().downElev();
    }
  }

  @Override
  public void end(boolean interrupted) {
    
    SubElev.getInstance().stopElev();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
