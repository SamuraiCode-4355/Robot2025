package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubShooter;

public class AutoShooter extends Command {

  private Timer crono;

  public AutoShooter() {

    addRequirements(SubShooter.getInstance());
    crono = new Timer();
  }

  @Override
  public void initialize() {

    crono.reset();
    crono.start();
  }

  @Override
  public void execute() {

    SubShooter.getInstance().shoot();
  }

  @Override
  public void end(boolean interrupted) {

    SubShooter.getInstance().stop();
    crono.stop();
    crono.reset();
  }

  @Override
  public boolean isFinished() {
    return crono.get() >= 1.2;
  }
}
