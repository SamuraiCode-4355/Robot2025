package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubSwerve;

public class ComSwerve extends Command {

  private DoubleSupplier leftX;
  private DoubleSupplier leftY;
  private DoubleSupplier rightX; 

  private double initialOrientation;
  private double velRotation;
  private PIDController rotationPID;

  public ComSwerve(DoubleSupplier LeftX, DoubleSupplier LeftY, DoubleSupplier RightX) {

    this.leftX=LeftX;
    this.leftY=LeftY;
    this.rightX=RightX;

    rotationPID = new PIDController(0.005, 0.0, 0.0);
    rotationPID.setTolerance(2.0);
    rotationPID.enableContinuousInput(0, 180);

    addRequirements(SubSwerve.getInstance());
  }

  @Override
  public void initialize() {

    initialOrientation = SubSwerve.getInstance().getHeading();
    rotationPID.reset();
    rotationPID.setSetpoint(initialOrientation);
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

    ChassisSpeeds newChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-leftY.getAsDouble(),
                                                                            -leftX.getAsDouble(),
                                                                            velRotation,
                                                                            SubSwerve.getInstance().robotOrientation());
    SubSwerve.getInstance().setChassisSpeed(newChassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {

    SubSwerve.getInstance().stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
