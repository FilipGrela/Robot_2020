/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

   public WPI_TalonSRX leftMaster, rightMaster;
   private WPI_VictorSPX  leftSlaveFront, leftSlaveBack, rightSlaveFront, rightSlaveBack;

  public DriveTrain() {
    leftMaster = new WPI_TalonSRX(PortMap.kLMasterDrive);
    rightMaster = new WPI_TalonSRX(PortMap.kRMasterDrive);

    leftSlaveFront = new WPI_VictorSPX(PortMap.kLSlaveMDrive);
    leftSlaveBack = new WPI_VictorSPX(PortMap.kRSlaveBDrive);
    rightSlaveFront = new WPI_VictorSPX(PortMap.kRSlaveMDrive);
    rightSlaveBack = new WPI_VictorSPX(PortMap.kRSlaveBDrive);

    configureMaster(leftMaster, false);
    configureMaster(rightMaster, false);

    leftSlaveBack.follow(leftMaster);
    leftSlaveFront.follow(leftMaster);
    rightSlaveBack.follow(leftMaster);
    rightSlaveFront.follow(leftMaster);
    stop();
  }

  public void stop(){
    setSpeed(0.0, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double leftSpeed, double rightSpeed){

    leftMaster.set(ControlMode.PercentOutput, leftSpeed);
    rightMaster.set(ControlMode.PercentOutput, rightSpeed);
  }


  
  public void configureMaster(WPI_TalonSRX talon, boolean invert){
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
    final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
            .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
    if (sensorPresent != ErrorCode.OK) {
        DriverStation.reportError("Could not detect " + (invert ? "right" : "left") + " encoder: " + sensorPresent, false);
    }

    //talon.setInverted();
    talon.setSensorPhase(true);
    talon.enableVoltageCompensation(true);
    talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
    talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
    talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
    talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
    talon.configNeutralDeadband(0.04, 0);
  }

  public double getRobotWheelsSpeedAverageRPS(){
    return ((getRobotLeftWheelsSpeedRPS()+getRobotRightWheelsSpeedRPS())/2);
  }  

  public double getRobotLeftWheelsSpeedRPS(){
    return (-(((double)leftMaster.getSelectedSensorVelocity()/4096)*10)/3);
  }  

  public double getRobotRightWheelsSpeedRPS(){
    return ((((double) rightMaster.getSelectedSensorVelocity()/4096)*10)/3);
  }  

  public void logsDriveTrain(){
    SmartDashboard.putNumber("Drive_Train_Left_Wheel_RPS", getRobotLeftWheelsSpeedRPS());
    SmartDashboard.putNumber("Drive_Train_Rignt_Wheel_RPS", getRobotRightWheelsSpeedRPS());
  }

}
