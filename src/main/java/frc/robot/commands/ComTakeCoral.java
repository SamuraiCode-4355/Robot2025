package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubLeds;

public class ComTakeCoral extends Command {

  public ComTakeCoral() {

    addRequirements(SubElev.getInstance());
  }

  @Override
  public void initialize() {

    SubElev.getInstance().suction();
    SubLeds.heartbeatBlue();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {

    SubElev.getInstance().stopShoot();
    SubLeds.turnOff();
  }

  @Override
  public boolean isFinished() {
    return !SubElev.getInstance().coral();
  }
}