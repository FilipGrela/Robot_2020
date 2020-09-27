/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.commands.CommonFunctions;

public class Drive extends CommandBase {
  /**
   * Creates a new Drive.
   */

  PIDController leftWheelController = new PIDController(Constants.openLoopkPLeft, Constants.openLoopkILeft, Constants.openLoopkDLeft);
  PIDController rightWheelController = new PIDController(Constants.openLoopkPLeft, Constants.openLoopkILeft, Constants.openLoopkDLeft);
 // PIDController rightWheelController = new PIDController(Constants.openLoopkPRight, Constants.openLoopkIRight, Constants.openLoopkDRight);

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

    leftWheelSpeed = 2*(CommonFunctions.eliminateDeadZone(leftDriverAxis, 0.1) - (rightDriverAxis/2));
    rightWheelSpeed = 2*(CommonFunctions.eliminateDeadZone(leftDriverAxis, 0.1) + (rightDriverAxis/2));


    
    Robot.driveTrain.setSpeed(leftWheelController.calculate(Robot.driveTrain.getRobotLeftWheelsSpeedRPS(), leftWheelSpeed),
                             rightWheelController.calculate(Robot.driveTrain.getRobotRightWheelsSpeedRPS() , rightWheelSpeed));

    logsDriveTrainCommand();
  }

  private void basicDrive(double leftDriverAxis, double rightDriverAxis){

    rightDriverAxis = CommonFunctions.eliminateDeadZone(rightDriverAxis, 0.1);

    leftWheelSpeed = CommonFunctions.eliminateDeadZone(leftDriverAxis, 0.1) - (rightDriverAxis/2);
    rightWheelSpeed = CommonFunctions.eliminateDeadZone(leftDriverAxis, 0.1) + (rightDriverAxis/2);

    Robot.driveTrain.setSpeed(leftWheelSpeed, rightWheelSpeed);
    logsDriveTrainCommand();
  }

  public void logsDriveTrainCommand(){
    SmartDashboard.putNumber("Drive_Train_Command_Target_Left", leftWheelSpeed);
    SmartDashboard.putNumber("Drive_Train_Command_Target_Right", rightWheelSpeed);
  }

}
