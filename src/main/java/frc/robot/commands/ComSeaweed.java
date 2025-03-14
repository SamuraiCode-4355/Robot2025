package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubIntake;

public class ComSeaweed extends Command {

  private BooleanSupplier button;
  private boolean level2;
  private Timer crono;

  public ComSeaweed(BooleanSupplier Button) {

    addRequirements(SubElev.getInstance(), SubIntake.getInstance());
    this.button = Button;
    crono = new Timer();
  }

  @Override
  public void initialize() {

    SubElev.getInstance().setLevel(3);
    SubElev.getInstance().enabledPID(true);
    crono.reset();
    crono.start();
  }

  @Override
  public void execute() {

    if(crono.get() >= 1){

      if(SubElev.getInstance().atSetPoint() && button.getAsBoolean() && !level2){

        SubIntake.getInstance().suction();
        SubElev.getInstance().setLevel(2);
        level2 = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {

    level2 = false;
    SubIntake.getInstance().stop();
    SubElev.getInstance().enabledPID(false);
    crono.stop();
    crono.reset();
  }

  @Override
  public boolean isFinished() {
    return level2 && SubElev.getInstance().atSetPoint();
  }
}
