package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.RobotConst;
import frc.robot.Constants.SwerveConts;
import frc.robot.math.Conversions;

public class SubSwerve extends SubsystemBase {

  private static SubSwerve instance;
  private SwerveModule m_FL;
  private SwerveModule m_BL;
  private SwerveModule m_FR;
  private SwerveModule m_BR;

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  private Pose2d initialPose;
  private Rotation2d initialOrientation;
  private Rotation2d robotOrientation;

  private ChassisSpeeds currentChassisState;
  private double robotVelocity;

  private Pigeon2 m_Gyro;
  private RobotConfig config;

  public SubSwerve() {

    m_FL = new SwerveModule(SwerveConts.KFLDriveID, SwerveConts.KFLTurnID, SwerveConts.KFLEncoderID,
      SwerveConts.KFLDriveInverted, SwerveConts.KFLTurnInverted, SwerveConts.KFLEncoderInverted);

    m_FR = new SwerveModule(SwerveConts.KFRDriveID, SwerveConts.KFRTurnID, SwerveConts.KFREncoderID, 
     SwerveConts.KFRDriveInverted, SwerveConts.KFRTurnInverted, SwerveConts.KFREncoderInverted);

    m_BL = new SwerveModule(SwerveConts.KBLDriveID,  SwerveConts.KBLTurnID, SwerveConts.KBLEncoderID, 
     SwerveConts.KBLDriveInverted, SwerveConts.KBLTurnInverted, SwerveConts.KBLEncoderInverted);

    m_BR = new SwerveModule(SwerveConts.KBRDriveID, SwerveConts.KBRTurnID, SwerveConts.KBREncoderID, 
     SwerveConts.KBRDriveInverted, SwerveConts.KBRTurnInverted, SwerveConts.KBREncoderInverted);

    kinematics = new SwerveDriveKinematics(RobotConst.m_FL_Location, RobotConst.m_FR_Location,
      RobotConst.m_BL_Location, RobotConst.m_BR_Location);

    m_Gyro = new Pigeon2(SwerveConts.KGiroID);

    initialPose = new Pose2d(7.994, 1.057, Rotation2d.fromDegrees(0));
    initialOrientation = Rotation2d.fromDegrees(0);

    try{
      
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {

      e.printStackTrace();
    }

    odometry = new SwerveDriveOdometry(kinematics, initialOrientation, 
      new SwerveModulePosition[]{
        new SwerveModulePosition(0, new Rotation2d()),
        new SwerveModulePosition(0, new Rotation2d()),
        new SwerveModulePosition(0, new Rotation2d()),
        new SwerveModulePosition(0, new Rotation2d())
      },
      initialPose
    );
    
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      () -> kinematics.toChassisSpeeds(getModuleStates()),
      this::setChassisSpeed,
      new PPHolonomicDriveController(
        new PIDConstants(1.35, 0.0, 0.0), //0.006  Traslado Desente -- 0.0006
        new PIDConstants(1, 0.0, 0.0)), //0.002 Rotancion Desente -- 0.00002
      config,
      () -> false,
      this 
    );  
  }

  public static SubSwerve getInstance(){

    if(instance == null){
      
      instance = new SubSwerve();
    }
    return instance;
  }

  public double getHeading(){

    return Math.IEEEremainder(m_Gyro.getYaw().getValueAsDouble(),360);
  }

  public Rotation2d robotOrientation(){

    robotOrientation = Rotation2d.fromDegrees(getHeading());
    return robotOrientation;
  }

  public double getHeadingReverse(){

    return Conversions.gyroReverse(getHeading());
  }

  public Rotation2d robotOrientationRev(){

    return Rotation2d.fromDegrees(getHeadingReverse());
  }

  public void resetGyro(){

    m_Gyro.reset();
  }

  public double getRobotVelocity(){

    currentChassisState = kinematics.toChassisSpeeds(m_FL.getActualState(), m_FR.getActualState(), 
                                                     m_BL.getActualState(), m_BR.getActualState());
                                                     
    robotVelocity = Math.hypot(currentChassisState.vyMetersPerSecond, currentChassisState.vxMetersPerSecond);
    return robotVelocity;
  }

  private SwerveModulePosition[] getModulePositions(){

    SwerveModulePosition[] modulePositions = {
      new SwerveModulePosition(m_FL.getMetersTraveled(), m_FL.getActualState().angle),
      new SwerveModulePosition(m_FR.getMetersTraveled(), m_FR.getActualState().angle),
      new SwerveModulePosition(m_BL.getMetersTraveled(), m_BL.getActualState().angle),
      new SwerveModulePosition(m_BR.getMetersTraveled(), m_BR.getActualState().angle)
    };
    return modulePositions;
  }

  private SwerveModuleState[] getModuleStates(){

    SwerveModuleState[] moduleStates = {
      m_FL.getActualState(),
      m_FR.getActualState(),
      m_BL.getActualState(),
      m_BR.getActualState(),
    };
    return moduleStates;
  }

  private Pose2d getPose(){

    return odometry.getPoseMeters();
  }

  private void resetPose(Pose2d pose){

    odometry.resetPosition(robotOrientationRev(), getModulePositions(), pose);
  }

  public void setBreak(){

    m_FL.setBreak();
    m_FR.setBreak();
    m_BL.setBreak();
    m_BR.setBreak();
  }

  public void setCoast(){

    m_FL.setCoast();
    m_FR.setCoast();
    m_BL.setCoast();
    m_BR.setCoast();
  }

  public void setChassisSpeed(ChassisSpeeds desiredSpeed){

    SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(desiredSpeed);

    m_FL.setDesiredState(newStates[0]);
    m_FR.setDesiredState(newStates[1]);
    m_BL.setDesiredState(newStates[2]);
    m_BR.setDesiredState(newStates[3]);
  }

  public void setDrive(double speed){

    m_FL.setDrive(speed);
    m_FR.setDrive(speed);
    m_BL.setDrive(speed);
    m_BR.setDrive(speed);
  }

  public void stop(){

    m_FL.stop();
    m_FR.stop();
    m_BL.stop();
    m_BR.stop();
  }

  @Override
  public void periodic() {

    if(!(odometry == null))
      odometry.update(robotOrientationRev(), getModulePositions());

    double SwerveStates[] = {
      m_FL.getActualState().angle.getRadians(),
      m_FL.getActualState().speedMetersPerSecond,
      m_FR.getActualState().angle.getRadians(),
      m_FR.getActualState().speedMetersPerSecond,
      m_BL.getActualState().angle.getRadians(),
      m_BL.getActualState().speedMetersPerSecond,
      m_BR.getActualState().angle.getRadians(),
      m_BR.getActualState().speedMetersPerSecond
    };

    SmartDashboard.putNumberArray("Swerve States", SwerveStates);
    SmartDashboard.putNumber("Gyro", getHeading());
    SmartDashboard.putNumber("Gyro Radians", Units.degreesToRadians(getHeading()));

    SmartDashboard.putNumber("FL speed", m_FL.getActualState().speedMetersPerSecond);
    SmartDashboard.putNumber("FR speed", m_FR.getActualState().speedMetersPerSecond);
    SmartDashboard.putNumber("BL speed", m_BL.getActualState().speedMetersPerSecond);
    SmartDashboard.putNumber("BR speed", m_BR.getActualState().speedMetersPerSecond);

    SmartDashboard.putNumber("FL SetP Speed", m_FL.getDesiredState().speedMetersPerSecond);
    SmartDashboard.putNumber("FR setP Speed", m_FR.getDesiredState().speedMetersPerSecond);
    SmartDashboard.putNumber("BL setP Speed", m_BL.getDesiredState().speedMetersPerSecond);
    SmartDashboard.putNumber("BR setP Speed", m_BR.getDesiredState().speedMetersPerSecond);

    SmartDashboard.putNumber("FL Voltage", m_FL.getVoltageDrive());
    SmartDashboard.putNumber("FR Voltage", m_FR.getVoltageDrive());
    SmartDashboard.putNumber("BL Voltage", m_BL.getVoltageDrive());
    SmartDashboard.putNumber("BR   Voltage", m_BR .getVoltageDrive());

    SmartDashboard.putNumber("Robot M/S", getRobotVelocity());

    if(!(LimelightHelpers.getSetP_Orientation("") == null))
      SmartDashboard.putNumber("SetP Orientation", LimelightHelpers.getSetP_Orientation(""));
  }
}
