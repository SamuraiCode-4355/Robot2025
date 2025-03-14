package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.Configure;
import frc.robot.subsystems.SubIntake;
import frc.robot.subsystems.SubSwerve;

public class ComArrangement extends Command {

  private ChassisSpeeds chassisSpeeds;
  private float horizontalSpeed = 0.3f;
  private boolean inverted;

  public ComArrangement() {

    addRequirements(SubSwerve.getInstance(), SubIntake.getInstance());
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    if(Configure.getSide() == 2 && !inverted){

      horizontalSpeed *= -1;
      inverted = true;
    }

    chassisSpeeds = new ChassisSpeeds(0.08, horizontalSpeed, 0.0);

    SubSwerve.getInstance().setChassisSpeed(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {

    SubSwerve.getInstance().stop();
    inverted = false;

    new ComShootCoral(false).schedule();
  }

  @Override
  public boolean isFinished() {
    return SubIntake.getInstance().reefArrangement();
  }
}
