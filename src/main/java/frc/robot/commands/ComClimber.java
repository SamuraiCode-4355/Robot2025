package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubClimber;
import frc.robot.subsystems.SubLeds;

public class ComClimber extends Command {

  private BooleanSupplier rightBumper;
  private BooleanSupplier leftBumper;
  private Timer crono;

  public ComClimber(BooleanSupplier RightBumper, BooleanSupplier LeftBumper) {

    addRequirements(SubClimber.getInstance());
    crono = new Timer();
    this.rightBumper = RightBumper;
    this.leftBumper = LeftBumper;
  }

  @Override
  public void initialize() {

    crono.reset();
    crono.start();
  }

  @Override
  public void execute() {

    if(rightBumper.getAsBoolean()){

      SubClimber.getInstance().retractClimber();
      crono.reset();
    }
    else if(leftBumper.getAsBoolean()){

      SubClimber.getInstance().takeOutClimber();
      crono.reset();
    }
    else{

      SubClimber.getInstance().stopClimber();
    }

    if(SubClimber.getInstance().robotUp())
      SubLeds.violet();
  }

  @Override
  public void end(boolean interrupted) {

    SubClimber.getInstance().stopClimber();
    SubLeds.turnOff();
  }

  @Override
  public boolean isFinished() {
    return crono.get() >= 3;
  }
}
