package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.Configure;
import frc.robot.math.Conversions;
import frc.robot.subsystems.SubSwerve;

public class ComCoordinates extends Command {

  private PIDController mPIDturn;
  private PIDController mPIDtx;
  private PIDController mPIDty;

  private float setPTurn;
  private float setPx;
  private float setPy;


  
  public ComCoordinates() {

    addRequirements(SubSwerve.getInstance());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {


    switch(Configure.getSideReef()){

      case 1: 
      System.out.print("Ir a coral");
      break;
      case 2:
      System.out.print("Se quemo el robot xd");
      break;
      case 3:
      System.out.print("Ganamos y nos vamos al mundial");
      break;
      case 4:
      System.out.print("Nos Despertamos y lloramos");
      break;
      case 5: 
      System.out.print("Pedro typec se burla de nosotros por no saber programar un PID");
      break;
      case 6: 
      System.out.print("Tecno ravens nos gana y va al mundial y lloramos");
      break;
      default :
      System.out.print("No hizo nada");
      break;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
