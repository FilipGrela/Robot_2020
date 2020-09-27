/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Flags;
import frc.robot.Robot;

public class IntakeMove extends CommandBase {
  /**
   * Creates a new IntakeMove.
   */
  public IntakeMove() {
    addRequirements(Robot.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(!Flags.getFlags().intakeDown){
      Robot.intake.intakeDown();
    }else if(Flags.getFlags().intakeDown){
      Robot.intake.intakeUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(Flags.getFlags().intakeDown){
      Flags.getFlags().intakeDown = false;
    }else if(!Flags.getFlags().intakeDown){
      Flags.getFlags().intakeDown = true;
    }
    Robot.intake.intakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
