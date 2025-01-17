package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SubSwerve;

public class ComAccommodation extends Command {

  private boolean finish;
  private String side;
  private ChassisSpeeds chassisSpeeds;

  private double ty;
  private double tx;
  private double ta;

  private double velX;
  private double velY;
  private double velTurn;

  private PIDController mPIDta;
  private PIDController mPIDtx;
  private PIDController mPIDturn;

  public ComAccommodation(String Side) {

    addRequirements(SubSwerve.getInstance());

    mPIDta = new PIDController(0.08, 0.0, 0.0);
    mPIDta.setTolerance(1);

    mPIDtx = new PIDController(0.015, 0.0, 0.0);
    mPIDtx.setTolerance(1);

    mPIDturn = new PIDController(0.005, 0.0, 0.0);
    mPIDturn.setTolerance(2);
    mPIDturn.enableContinuousInput(0, 180);

    this.side = Side;
  }

  @Override
  public void initialize() {

    mPIDta.reset();
    mPIDtx.reset();
    mPIDturn.reset();

    if(side == "LEFT"){

      mPIDta.setSetpoint(5.45);
      mPIDtx.setSetpoint(9.8);
      LimelightHelpers.setLEDMode_PipelineControl("");
      LimelightHelpers.setLEDMode_ForceBlink("");

    }
    else if(side == "RIGHT"){

      mPIDta.setSetpoint(5.6);
      mPIDtx.setSetpoint(-18);
      LimelightHelpers.setLEDMode_PipelineControl("");
      LimelightHelpers.setLEDMode_ForceBlink("");

    }
    else{

      finish = true;
      DriverStation.reportError("the \"ComAcom\" side is not set correctly, it was set to " + side + "instead of \"LEFT\" or \"RIGHT\".", false);
    }

    if(LimelightHelpers.getSetP_Orientation("") == null){

      finish = true;
    }
    else{

      mPIDturn.setSetpoint(LimelightHelpers.getSetP_Orientation(""));
    }
  }

  @Override
  public void execute() {

    ty = LimelightHelpers.getTY("");
    tx = LimelightHelpers.getTX("");
    ta = LimelightHelpers.getTA("");

    velX = mPIDta.calculate(ta);
    velY = mPIDtx.calculate(tx);
    velTurn = mPIDturn.calculate(SubSwerve.getInstance().getHeading());

    chassisSpeeds = new ChassisSpeeds(velX, velY, velTurn);

    SubSwerve.getInstance().setChassisSpeed(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {

    SubSwerve.getInstance().stop();
    finish = false;
  }

  @Override
  public boolean isFinished() {
    return (ta == 0 && tx == 0 && ty == 0) || (mPIDtx.atSetpoint() && mPIDta.atSetpoint()) || finish;
  }
}
