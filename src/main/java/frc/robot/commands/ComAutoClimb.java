package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubClimber;

public class ComAutoClimb extends Command {

  private Timer crono;

  public ComAutoClimb() {

    addRequirements(SubClimber.getInstance());
    crono = new Timer();
  }
 
  @Override
  public void initialize() {

    crono.reset();
    crono.start();
    SubClimber.getInstance().takeOutClimber();
  }


  @Override
  public void execute() {

  }
    

  @Override
  public void end(boolean interrupted) {

    SubClimber.getInstance().stopClimber();
    crono.stop();
    crono.reset();
  }

  @Override
  public boolean isFinished() {
    return crono.get() >= 0.5;
  }
}
