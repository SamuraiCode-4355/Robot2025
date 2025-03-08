package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.Configure;
import frc.robot.subsystems.SubIntake;
import frc.robot.subsystems.SubSwerve;

public class ComArrangement extends Command {

  private ChassisSpeeds chassisSpeeds;
  private float horizontalSpeed = 0.3f;

  public ComArrangement() {

    addRequirements(SubSwerve.getInstance(), SubIntake.getInstance());

    if(Configure.getSide() == 2)
      horizontalSpeed *= -1;

    chassisSpeeds = new ChassisSpeeds(0.0, horizontalSpeed, 0.0);
  }

  @Override
  public void initialize() {

    SubSwerve.getInstance().setChassisSpeed(chassisSpeeds);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {

    SubSwerve.getInstance().stop();

    new ComShootCoral().schedule();
  }

  @Override
  public boolean isFinished() {
    return SubIntake.getInstance().reefArrangement();
  }
}
