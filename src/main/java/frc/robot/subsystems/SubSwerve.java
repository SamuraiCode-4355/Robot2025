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
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.math.Conversions;

public class SubSwerve extends SubsystemBase {

  private static SubSwerve instance;
  private SwerveModule m_FL;
  private SwerveModule m_BL;
  private SwerveModule m_FR;
  private SwerveModule m_BR;

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  //private Pose2d initialPose;
  //private Rotation2d initialOrientation;
  private Rotation2d robotOrientation;

  private ChassisSpeeds currentChassisState;
  private double robotVelocity;
 // private double[] robotPose;

  private Pigeon2 m_Gyro;
  private RobotConfig config;
  private double setPSpeed;

  public SubSwerve() {

    m_FL = new SwerveModule(SwerveConstants.kFLDriveID, SwerveConstants.kFLTurnID, SwerveConstants.kFLEncoderID,
      SwerveConstants.kFLDriveInverted, SwerveConstants.kFLTurnInverted, SwerveConstants.kFLEncoderInverted);

    m_FR = new SwerveModule(SwerveConstants.kFRDriveID, SwerveConstants.kFRTurnID, SwerveConstants.kFREncoderID, 
     SwerveConstants.kFRDriveInverted, SwerveConstants.kFRTurnInverted, SwerveConstants.kFREncoderInverted);

    m_BL = new SwerveModule(SwerveConstants.kBLDriveID,  SwerveConstants.kBLTurnID, SwerveConstants.kBLEncoderID, 
     SwerveConstants.kBLDriveInverted, SwerveConstants.kBLTurnInverted, SwerveConstants.kBLEncoderInverted);

    m_BR = new SwerveModule(SwerveConstants.kBRDriveID, SwerveConstants.kBRTurnID, SwerveConstants.kBREncoderID, 
     SwerveConstants.kBRDriveInverted, SwerveConstants.kBRTurnInverted, SwerveConstants.kBREncoderInverted);

    kinematics = new SwerveDriveKinematics(RobotConstants.kFL_Location, RobotConstants.kFR_Location,
      RobotConstants.kBL_Location, RobotConstants.kBR_Location);

    m_Gyro = new Pigeon2(SwerveConstants.kGyroID);

   // initialPose = new Pose2d(7.994, 1.057, Rotation2d.fromDegrees(0));
    //initialOrientation = Rotation2d.fromDegrees(0);

    try{
      
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {

      e.printStackTrace();
    }

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(),//initialOrientation, 
      new SwerveModulePosition[]{
        new SwerveModulePosition(0, new Rotation2d()),
        new SwerveModulePosition(0, new Rotation2d()),
        new SwerveModulePosition(0, new Rotation2d()),
        new SwerveModulePosition(0, new Rotation2d())
      },
      new Pose2d()
    );
    
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      () -> kinematics.toChassisSpeeds(getModuleStates()),
      this::setChassisSpeed,
      new PPHolonomicDriveController(
        new PIDConstants(2.8, 0.0, 0.0),  //1.8
        new PIDConstants(3.8, 0.0, 0.0)), //2.8
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

    setPSpeed = m_FL.getDesiredState().speedMetersPerSecond +
                m_FR.getDesiredState().speedMetersPerSecond +
                m_BL.getDesiredState().speedMetersPerSecond +
                m_BR.getDesiredState().speedMetersPerSecond;

    if(setPSpeed < 0.05)
      stop();

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

    /*if(!(LimelightHelpers.getSetP_Orientation("") == null))
      SmartDashboard.putNumber("SetP Orientation", LimelightHelpers.getSetP_Orientation(""));

    robotPose = LimelightHelpers.getBotPose("");

    if(!(robotPose == null)){

      SmartDashboard.putNumber("x Pose", robotPose[0]);
      SmartDashboard.putNumber("y Pose", robotPose[1]);
    }*/
  }
}
