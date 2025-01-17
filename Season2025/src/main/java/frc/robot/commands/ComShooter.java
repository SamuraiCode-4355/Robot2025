package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubShooter;

public class ComShooter extends Command {

  public ComShooter() {

    addRequirements(SubShooter.GetInstance());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    SubShooter.GetInstance().Shoot();
  }

  @Override
  public void end(boolean interrupted) {

    SubShooter.GetInstance().Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
