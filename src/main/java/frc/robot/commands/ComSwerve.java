package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubSwerve;

public class ComSwerve extends Command {

  private DoubleSupplier leftX;
  private DoubleSupplier leftY;
  private DoubleSupplier rightX; 

  private double initialOrientation;
  private double velRotation;
  private PIDController rotationPID;
  private ChassisSpeeds chassisSpeeds;
  private boolean downElev;

  public ComSwerve(DoubleSupplier LeftX, DoubleSupplier LeftY, DoubleSupplier RightX) {

    this.leftX = LeftX;
    this.leftY = LeftY;
    this.rightX = RightX;

    rotationPID = new PIDController(0.025, 0.0, 0.0);
    rotationPID.setTolerance(1);
    rotationPID.enableContinuousInput(0, 180);

    addRequirements(SubSwerve.getInstance());//, SubElev.getInstance());
  }

  @Override
  public void initialize() {

    initialOrientation = SubSwerve.getInstance().getHeading();
    rotationPID.reset();
    rotationPID.setSetpoint(initialOrientation);
/* 
    if(SubElev.getInstance().coralShooting() && SubElev.getInstance().isUpElev()){

      SubSwerve.getInstance().resetMetersTraveled();
      downElev = false;
    }
    else{

      downElev = true;
    }*/
  }

  @Override
  public void execute() {

    if(Math.abs(rightX.getAsDouble()) < 0.08){

      velRotation = rotationPID.calculate(SubSwerve.getInstance().getHeading());
    }
    else{

      velRotation = -rightX.getAsDouble();
      initialOrientation = SubSwerve.getInstance().getHeading();
      rotationPID.setSetpoint(initialOrientation);
    }

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-leftY.getAsDouble(), -leftX.getAsDouble(), velRotation,
    SubSwerve.getInstance().robotOrientation());
    /*(SubElev.getInstance().isUpElev() && !SubElev.getInstance().coralShooting()) ? 
        SubSwerve.getInstance().frontRobot() : SubSwerve.getInstance().robotOrientation());*/
    
    SubSwerve.getInstance().setChassisSpeed(chassisSpeeds);


   /*  if(Math.abs(SubSwerve.getInstance().getFL_MTraveled()) >= 0.4 && !downElev){

      new ComDownElev().schedule();
      downElev = true;
    }*/
  }

  @Override
  public void end(boolean interrupted) {

    SubSwerve.getInstance().stop();
    downElev = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
