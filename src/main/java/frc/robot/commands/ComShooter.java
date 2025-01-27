package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubLeds;
import frc.robot.subsystems.SubShooter;

public class ComShooter extends Command {

  private BooleanSupplier rightB;

  public ComShooter(BooleanSupplier RightB) {

    addRequirements(SubShooter.getInstance(), SubLeds.getInstance());
    this.rightB = RightB;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if(rightB.getAsBoolean()){

      SubShooter.getInstance().shoot();
      SubLeds.getInstance().ledBlue();
    }
    else{

      SubShooter.getInstance().suction();
    }
  }

  @Override
  public void end(boolean interrupted) {

    SubShooter.getInstance().stop();
    SubLeds.getInstance().ledOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
