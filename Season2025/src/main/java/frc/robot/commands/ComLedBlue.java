

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubLeds;
import frc.robot.subsystems.SubSwerve;

public class ComLedBlue extends Command {

  private Timer crono;

  public ComLedBlue() {

    addRequirements(SubLeds.getInstance(), SubSwerve.getInstance());
    crono = new Timer();
  }

  @Override
  public void initialize() {

    crono.reset();
    crono.start();
    SubLeds.getInstance().ledBlue();
  }

  @Override
  public void execute() {

    SubSwerve.getInstance().stop();
  }

  @Override
  public void end(boolean interrupted) {

    SubLeds.getInstance().ledOff();
    crono.stop();
    crono.reset();
  }

  @Override
  public boolean isFinished() {
    return crono.get() >= 1.5;
  }
}
