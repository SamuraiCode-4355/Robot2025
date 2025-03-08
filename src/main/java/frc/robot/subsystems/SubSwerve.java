package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
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
  private Rotation2d robotOrientation;

  private double setPSpeed;
  private double[] swerveStates;
  private SwerveModulePosition[] modulePositions;

  private Pigeon2 m_Gyro;
  private RobotConfig config;
  private ColorSensorV3 mColorSensorV3;

  //-------------------------MÉTODO CONSTRUCTOR--------------------------

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

    try{
      
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {

      e.printStackTrace();
    }

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(),
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
        new PIDConstants(2.2, 0.0, 0.0),  //2.8
        new PIDConstants(3.0 , 0.0, 0.0)), //3.8
      config,
      () -> RobotConstants.redAlliance,
      this 
    );  

    mColorSensorV3 = new ColorSensorV3(I2C.Port.kMXP);
    swerveStates = new double[8];
    modulePositions = new SwerveModulePosition[4];
  }

  //---------------MÉTODO DE FÁBRICA---------------------

  public static SubSwerve getInstance(){

    if(instance == null){
      
      instance = new SubSwerve();
    }
    return instance;
  }

//---------------------ATRIBUTOS----------------------------

  public Rotation2d robotOrientation(){

    robotOrientation = Rotation2d.fromDegrees(getHeading());
    return robotOrientation;
  }

  public Rotation2d robotOrientationRev(){

    return Rotation2d.fromDegrees(getHeadingReverse());
  }
  
  public Rotation2d frontRobot(){

    return Rotation2d.fromDegrees(0.0);
  }

  public double getHeading(){

    return Math.IEEEremainder(m_Gyro.getYaw().getValueAsDouble(),360);
  }

  public double getHeadingReverse(){

    return Conversions.gyroReverse(getHeading());
  }

  private SwerveModulePosition[] getModulePositions(){

    modulePositions[0] = new SwerveModulePosition(m_FL.getMetersTraveled(), m_FL.getCurrentState().angle);
    modulePositions[1] = new SwerveModulePosition(m_FR.getMetersTraveled(), m_FR.getCurrentState().angle);
    modulePositions[2] = new SwerveModulePosition(m_BL.getMetersTraveled(), m_BL.getCurrentState().angle);
    modulePositions[3] = new SwerveModulePosition(m_BR.getMetersTraveled(), m_BR.getCurrentState().angle);
    return modulePositions;
  }

  private SwerveModuleState[] getModuleStates(){

    SwerveModuleState[] moduleStates = {
      m_FL.getCurrentState(),
      m_FR.getCurrentState(),
      m_BL.getCurrentState(),
      m_BR.getCurrentState(),
    };
    return moduleStates;
  }

  private Pose2d getPose(){

    return odometry.getPoseMeters();
  }

  public boolean coralStation(){

    return mColorSensorV3.getProximity() >= SwerveConstants.kCoralStationSensor;
  }

  public boolean isUpElev(){

    return SubClimber.getInstance().getEncoderElev() >= 1.5;
  }

  //----------------------------MÉTODOS--------------------------------

  public void resetGyro(){

    m_Gyro.reset();
  }

  private void resetPose(Pose2d pose){

    odometry.resetPosition(robotOrientationRev(), getModulePositions(), pose);
  }

  public void resetMetersTraveled(){

    m_FL.resetMetersTraveled();
    m_FR.resetMetersTraveled();
    m_BL.resetMetersTraveled();
    m_BR.resetMetersTraveled();
  }

  public void setChassisSpeed(ChassisSpeeds desiredSpeed){

    SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(desiredSpeed);

    m_FL.setDesiredState(newStates[0]);
    m_FR.setDesiredState(newStates[1]);
    m_BL.setDesiredState(newStates[2]);
    m_BR.setDesiredState(newStates[3]);
  }

  public void stop(){

    m_FL.stop();
    m_FR.stop();
    m_BL.stop();
    m_BR.stop();
  }

  //----------------------MÉTODO PERIODICO--------------------------------

  @Override
  public void periodic() {

    if(odometry != null)
      odometry.update(robotOrientationRev(), getModulePositions());

    swerveStates[0] = m_FL.getCurrentState().angle.getRadians();
    swerveStates[1] = m_FL.getCurrentState().speedMetersPerSecond;
    swerveStates[2] = m_FR.getCurrentState().angle.getRadians();
    swerveStates[3] = m_FR.getCurrentState().speedMetersPerSecond;
    swerveStates[4] = m_BL.getCurrentState().angle.getRadians();
    swerveStates[5] = m_BL.getCurrentState().speedMetersPerSecond;
    swerveStates[6] = m_BR.getCurrentState().angle.getRadians();
    swerveStates[7] = m_BR.getCurrentState().speedMetersPerSecond;

    setPSpeed = Math.abs(m_FL.getDesiredState().speedMetersPerSecond) +
                Math.abs(m_FR.getDesiredState().speedMetersPerSecond) +
                Math.abs(m_BL.getDesiredState().speedMetersPerSecond) +
                Math.abs(m_BR.getDesiredState().speedMetersPerSecond);

    if(setPSpeed < 0.08)
      stop();

    SmartDashboard.putNumberArray("Swerve States", swerveStates);
    SmartDashboard.putNumber("Gyro", getHeading());
    SmartDashboard.putNumber("Gyro Radians", Units.degreesToRadians(getHeading()));
  }
}
