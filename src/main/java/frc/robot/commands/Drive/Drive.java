/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.commands.CommonFunctions;

public class Drive extends CommandBase {
  /**
   * Creates a new Drive.
   */

  PIDController leftWheelController = new PIDController(Constants.openLoopkPLeft, Constants.openLoopkILeft, Constants.openLoopkDLeft);
  PIDController rightWheelController = new PIDController(Constants.openLoopkPRight, Constants.openLoopkIRight, Constants.openLoopkDRight);

  double leftWheelSpeed;
  double rightWheelSpeed;

  public Drive() {
    addRequirements(Robot.driveTrain);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //basicDrive(Robot.oi.getDriverJoystick().getRawAxis(PortMap.kSpeedAxisPort), Robot.oi.getDriverJoystick().getRawAxis(PortMap.kTurnAxisPort));
    driveWithGradualAcceleration(Robot.oi.getDriverJoystick().getRawAxis(PortMap.kSpeedAxisPort), Robot.oi.getDriverJoystick().getRawAxis(PortMap.kTurnAxisPort));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void driveWithGradualAcceleration(double leftDriverAxis, double rightDriverAxis){

    rightDriverAxis = CommonFunctions.eliminateDeadZone(rightDriverAxis, 0.1);

    leftWheelSpeed = CommonFunctions.eliminateDeadZone(leftDriverAxis, 0.1) + rightDriverAxis;
    rightWheelSpeed = CommonFunctions.eliminateDeadZone(leftDriverAxis, 0.1) - rightDriverAxis;
    
    Robot.driveTrain.setSpeed(leftWheelController.calculate(Robot.driveTrain.getRobotLeftWheelsSpeedRPS(),leftWheelSpeed),
                             rightWheelController.calculate(Robot.driveTrain.getRobotLeftWheelsSpeedRPS() ,rightWheelSpeed));

  }

  private void basicDrive(double leftDriverAxis, double rightDriverAxis){

    leftDriverAxis = CommonFunctions.eliminateDeadZone(leftDriverAxis, Constants.joyDeadZone);
    rightDriverAxis = CommonFunctions.eliminateDeadZone(rightDriverAxis, Constants.joyDeadZone);

    Robot.driveTrain.setSpeed(leftDriverAxis-rightDriverAxis, leftDriverAxis+rightDriverAxis);
  }

}
