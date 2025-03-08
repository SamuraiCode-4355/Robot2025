package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubClimber;

public class ComAutoClimb extends Command {

  private BooleanSupplier FinishButton;

  public ComAutoClimb(BooleanSupplier finishbutton) {

    this.FinishButton = finishbutton;
    addRequirements(SubClimber.getInstance());
  }
 
  @Override
  public void initialize() {

    SubClimber.getInstance().retractClimber();
  }


  @Override
  public void execute() {

    if(SubClimber.getInstance().isRobotUp()){

      SubClimber.getInstance().takeOutClimber();
    }
  }
    

  @Override
  public void end(boolean interrupted) {

    SubClimber.getInstance().stopClimber();
  }

  @Override
  public boolean isFinished() {
    return FinishButton.getAsBoolean();
  }
}
