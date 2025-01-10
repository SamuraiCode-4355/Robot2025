package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SubSwerve;

public class ComContinueL extends Command {

  private double ta;
  private double tx;
  private double ty;

  private PIDController mPidta;
  private PIDController mPidtx;

  private double velTY;
  private double velTX;

  public ComContinueL() {
    addRequirements(SubSwerve.getInstance());
    
    mPidta = new PIDController(0.0, 0.0, 0.0);
    mPidta.setTolerance(0.0);

   
    mPidtx = new PIDController(0.0, 0.0, 0.0);
    mPidtx.setTolerance(0.0);
  }

  @Override
  public void initialize() {
    mPidta.reset();
    mPidta.setSetpoint(0.0);

    mPidtx.reset();
    mPidtx.setSetpoint(0.0);
  }

  @Override
  public void execute() {
    ta = LimelightHelpers.getTA("");
    tx = LimelightHelpers.getTX("");
    ty = LimelightHelpers.getTY("");
    
    velTX = mPidtx.calculate(ta);
    velTY = mPidta. calculate(tx);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(velTX, velTY, 0.0,
    SubSwerve.getInstance().robotOrientation());
    SubSwerve.getInstance().setChassisSpeed(chassisSpeeds);
  }

  @Override
    public void end(boolean interrupted) {
      SubSwerve.getInstance().stop();  
    }

  @Override
  public boolean isFinished() {
    return (ta == 0 && tx == 0 && ty == 0) || mPidta.atSetpoint() && mPidtx.atSetpoint() ;
  }
}
