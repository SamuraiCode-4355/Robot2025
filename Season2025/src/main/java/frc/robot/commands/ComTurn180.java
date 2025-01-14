package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubSwerve;

public class ComTurn180 extends Command {

  private Timer crono;
  private double setPoint;
  private double orientation;

  private double turnVel;
  private double xVel;
  private PIDController mPID;
  private ChassisSpeeds chassisSpeeds;

  public ComTurn180() {

    addRequirements(SubSwerve.getInstance());
    mPID = new PIDController(0.03, 0.0, 0.0);
    mPID.setTolerance(2);
    crono = new Timer();
  }

  @Override
  public void initialize() {

    orientation = SubSwerve.getInstance().getHeading();
    setPoint = orientation += (orientation < 0) ? 180 : -180;
    mPID.reset();
    mPID.setSetpoint(setPoint);

    crono.reset();
    crono.start();
  }

  @Override
  public void execute() {

    turnVel = (crono.get() <= 0.4) ? 0.0 : mPID.calculate(SubSwerve.getInstance().getHeading());
    xVel = (crono.get() <= 0.7) ? -0.5 : 0.0;

    chassisSpeeds = new ChassisSpeeds(xVel, 0.0, turnVel);
    SubSwerve.getInstance().setChassisSpeed(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {

    SubSwerve.getInstance().stop();
    crono.stop();
    crono.reset();
  }

  @Override
  public boolean isFinished() {
    return mPID.atSetpoint();
  }
}
