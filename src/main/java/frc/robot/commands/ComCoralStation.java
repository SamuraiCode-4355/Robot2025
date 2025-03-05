package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubSwerve;

public class ComCoralStation extends Command {

  private ChassisSpeeds chassisSpeeds;
  private boolean finish;
  private boolean arrangement;

  public ComCoralStation() {

    addRequirements(SubSwerve.getInstance(), SubElev.getInstance());
    chassisSpeeds = new ChassisSpeeds(-0.3, 0.0, 0.0);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if(!arrangement){

      SubSwerve.getInstance().setChassisSpeed(chassisSpeeds);
      if(SubSwerve.getInstance().coralStation())
        arrangement = true;
    }
    else{

      SubSwerve.getInstance().stop();  
      if(SubElev.getInstance().coral())
        finish = true;
    }
  }

  @Override
  public void end(boolean interrupted) {

    SubSwerve.getInstance().stop();
    finish = false;
    arrangement = false;
    new ComTakeCoral().schedule();
  }

  @Override
  public boolean isFinished() {
    return finish;
  }
}
