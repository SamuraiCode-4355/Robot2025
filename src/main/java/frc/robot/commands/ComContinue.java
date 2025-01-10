package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SubSwerve;

public class ComContinue extends Command {

  private double ty;
  private double tx;
  private double ta;

  private double velX;
  private double velY;

  private PIDController mPIDta;
  private PIDController mPIDtx;

  public ComContinue() {

    addRequirements(SubSwerve.getInstance());
    mPIDta = new PIDController(0.1, 0.0, 0.0);
    mPIDtx = new PIDController(0.05, 0.0, 0.0);

    mPIDta.setTolerance(2);
    mPIDtx.setTolerance(2);
  }

  @Override
  public void initialize() {

    mPIDta.reset();
    mPIDta.setSetpoint(14);

    mPIDtx.reset();
    mPIDtx.setSetpoint(0.0);
  }

  @Override
  public void execute() {

    ty = LimelightHelpers.getTY("");
    tx = LimelightHelpers.getTX("");
    ta = LimelightHelpers.getTA("");

    velX = mPIDta.calculate(ta);
    velY = mPIDtx.calculate(tx);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(velX, velY, 
                                  0, SubSwerve.getInstance().robotOrientation());

    SubSwerve.getInstance().setChassisSpeed(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {

    SubSwerve.getInstance().stop();
  }

  @Override
  public boolean isFinished() {
    return (ta == 0 && tx == 0 && ty == 0) || mPIDtx.atSetpoint() && mPIDta.atSetpoint();
  }
}
