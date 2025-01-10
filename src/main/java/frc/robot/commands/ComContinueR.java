package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SubSwerve;

public class ComContinueR extends Command {
  
  private double tx;
  private double ta;
  private double ty;
  
  private PIDController mPidta;
  private PIDController mPidtx;

  private double Veltx;
  private double Velty;

  public ComContinueR() {
    
    addRequirements(SubSwerve.getInstance());
    mPidta=new PIDController(0.1, 0.0, 0.0);
    mPidta.setTolerance(2);

    mPidtx=new PIDController(0.05, 0.0, 0.0);
    mPidtx.setTolerance(2);
  }

  @Override
  public void initialize() {
    mPidta.reset();
    mPidta.setSetpoint(9);


    mPidtx.reset();
    mPidtx.setSetpoint(-20);
  }

  @Override
  public void execute() {
    ta = LimelightHelpers.getTA("");
    tx = LimelightHelpers.getTX("");
    ty = LimelightHelpers.getTY(""); 

    Veltx = mPidtx.calculate(tx);
    Velty= mPidta.calculate(ta);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(Veltx, Velty, 0.0,
    SubSwerve.getInstance().robotOrientation());
    SubSwerve.getInstance().setChassisSpeed(chassisSpeeds);


  }

  @Override
  public void end(boolean interrupted) {
    SubSwerve.getInstance().stop();
  }

  @Override
  public boolean isFinished() {
    return (ta ==0 && tx==0 && ty==0) || mPidta.atSetpoint() && mPidtx.atSetpoint();
  }
}
