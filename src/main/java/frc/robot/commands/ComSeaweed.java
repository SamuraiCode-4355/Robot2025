package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubIntake;

public class ComSeaweed extends Command {

  private BooleanSupplier button;
  private boolean level2;

  public ComSeaweed(BooleanSupplier Button) {

    addRequirements(SubElev.getInstance(), SubIntake.getInstance());
    this.button = Button;
  }

  @Override
  public void initialize() {

    SubElev.getInstance().setLevel(3);
    SubElev.getInstance().enabledPID(true);
  }

  @Override
  public void execute() {

    if(SubElev.getInstance().atSetPoint() && button.getAsBoolean() && !level2){

      SubIntake.getInstance().suction();
      SubElev.getInstance().setLevel(2);
      level2 = true;
    }
  }

  @Override
  public void end(boolean interrupted) {

    level2 = false;
    SubIntake.getInstance().stop();
    SubElev.getInstance().enabledPID(false);
  }

  @Override
  public boolean isFinished() {
    return level2 && SubElev.getInstance().atSetPoint();
  }
}
