package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.math.Configure;
import frc.robot.math.Conversions;
import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubSwerve;

public class ComArrangement extends Command {

  private boolean atSetPoint;
  private boolean readyToShoot;
  private boolean finish;
  private boolean aprilTagDetected = true;
  private int config;
  private ChassisSpeeds chassisSpeeds;
  private Timer crono;

  private double ty;
  private double tx;
  private double ta;

  private double velX;
  private double velY;
  private double velTurn;

  private PIDController mPIDta;
  private PIDController mPIDtx;
  private PIDController mPIDturn;

  public ComArrangement(int Config) {

    addRequirements(SubSwerve.getInstance(), SubElev.getInstance());

    mPIDta = new PIDController(0.07, 0.0, 0.0);//0.05
    mPIDta.setTolerance(1);

    mPIDtx = new PIDController(0.05, 0.0, 0.15);//0.03
    mPIDtx.setTolerance(1);

    mPIDturn = new PIDController(0.025, 0.0, 0.0);//0.025
    mPIDturn.setTolerance(1);
    mPIDturn.enableContinuousInput(0, 180);
    crono = new Timer();

    this.config = Config;
  }

  @Override
  public void initialize() {

    mPIDta.reset();
    mPIDtx.reset();
    mPIDturn.reset();

    crono.reset();

    LimelightHelpers.setLEDMode_ForceOn("");

    if(config == 0){

      if(Configure.getSide() == 1){

        mPIDta.setSetpoint(FieldConstants.kTaLeft);
        mPIDtx.setSetpoint(FieldConstants.kTxLeft);
      }
      else if(Configure.getSide() == 2){

        mPIDta.setSetpoint(FieldConstants.kTaRight);
        mPIDtx.setSetpoint(FieldConstants.kTxRight);
      }
      else{

        finish = true;
      }
    }
    else if(config == 1){

      mPIDta.setSetpoint(FieldConstants.kTaLeft);
      mPIDtx.setSetpoint(FieldConstants.kTxLeft);
    }
    else{

      mPIDta.setSetpoint(FieldConstants.kTaRight);
      mPIDtx.setSetpoint(FieldConstants.kTxRight);
    }

    if(LimelightHelpers.getSetP_Orientation("") != null){

      if(config > 0){

        mPIDturn.setSetpoint(Conversions.gyroReverse(LimelightHelpers.getSetP_Orientation("")));
      }
      else{

        mPIDturn.setSetpoint(LimelightHelpers.getSetP_Orientation(""));
      }
    }

    SubElev.getInstance().setLevel(Configure.getLevel());
    SubElev.getInstance().enabledPID(true);
  }

  @Override
  public void execute() {

    ty = LimelightHelpers.getTY("");
    tx = LimelightHelpers.getTX("");
    ta = LimelightHelpers.getTA("");

    velTurn = mPIDturn.calculate(SubSwerve.getInstance().getHeading());

    if(mPIDtx.atSetpoint() && mPIDta.atSetpoint() && !atSetPoint){

      SubSwerve.getInstance().resetMetersTraveled();
      atSetPoint = true;
    }

    if(!atSetPoint){

      velX = mPIDta.calculate(ta);
      velY = mPIDtx.calculate(tx);
      if(ta == 0 && tx == 0 && ty == 0)
        aprilTagDetected = false;
    }
    else{

      finish = true;
/* 
      if(Math.abs(SubSwerve.getInstance().getFL_MTraveled()) < FieldConstants.kDistanceFront){

        velX = 0.3;
        velY = 0.0;
      }
      else{

        velX = 0.0;

        if(!readyToShoot && SubElev.getInstance().atSetPoint()){

          crono.start();
          readyToShoot = true;
        }
       
        if(readyToShoot){

          SubElev.getInstance().shoot(0.95);
        }
      }*/
    }

    chassisSpeeds = new ChassisSpeeds(velX, velY, velTurn);

    SubSwerve.getInstance().setChassisSpeed(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {

    SubSwerve.getInstance().stop();
    LimelightHelpers.setLEDMode_ForceOff("");

    finish = false;
    atSetPoint = false;
    readyToShoot = false;
    aprilTagDetected = true;


    SubElev.getInstance().enabledPID(false);
    SubElev.getInstance().stopElev();
    SubElev.getInstance().stopShoot();

    crono.stop();
    crono.reset();

    new ComDownElev().schedule();
  }

  @Override
  public boolean isFinished() {
    return !aprilTagDetected || finish || crono.get() >= 1.2;
  }
}
