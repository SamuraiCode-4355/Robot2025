package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.math.Configure;
import frc.robot.subsystems.SubSwerve;

public class ComCoordinates extends Command {

  private boolean finish;
  private PIDController mPIDx;
  private PIDController mPIDy;
  private PIDController mPIDturn;

  private float setPTurn;
  private float setPx;
  private float setPy;

  private double[] currentCoord;

  public ComCoordinates() {

    addRequirements(SubSwerve.getInstance());

    mPIDx = new PIDController(0, 0, 0);
    mPIDy = new PIDController(0, 0, 0);
    mPIDturn = new PIDController(0, 0, 0);

    mPIDx.setTolerance(0);
    mPIDy.setTolerance(0);
    mPIDturn.setTolerance(0);

    currentCoord = new double[2];
  }

  @Override
  public void initialize() {

    currentCoord =  LimelightHelpers.getBotPose("");
    if(Math.abs(currentCoord[0]) < 1)
      finish = true;
  }

  @Override
  public void execute() {

    if(Math.abs(currentCoord[0] - LimelightHelpers.getBotPose("")[0]) < 1){

      currentCoord[0] = LimelightHelpers.getBotPose("")[0];
    }


    switch(Configure.getSideReef()){

      case 1: 

      break;
      case 2:

      break;
      case 3:

      break;
      case 4:

      break;
      case 5: 

      break;
      case 6: 

      break;
      default:
        finish = true;
      break;
    }
  }

  @Override
  public void end(boolean interrupted) {

    SubSwerve.getInstance().stop();
    finish = false;
  }

  @Override
  public boolean isFinished() {
    return finish;
  }
}
