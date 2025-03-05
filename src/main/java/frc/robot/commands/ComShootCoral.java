// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.Configure;
import frc.robot.subsystems.SubElev;
import frc.robot.subsystems.SubIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ComShootCoral extends Command {

  private Timer crono;

  public ComShootCoral() {

    addRequirements(SubElev.getInstance(), SubIntake.getInstance());
    crono = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    crono.reset();
    crono.start();

    if(Configure.getLevel() == 1){

      SubElev.getInstance().shoot(-0.2);//-0.3
      SubIntake.getInstance().shoot(0.3);
    }
    else{

      SubElev.getInstance().shoot(-0.45);//-0.3
      SubIntake.getInstance().shoot(0.15);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    SubElev.getInstance().stopShoot();
    SubElev.getInstance().isCoralShooting(true);
    SubIntake.getInstance().stop();

    crono.stop();
    crono.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return crono.get() >= 0.7;
  }
}
