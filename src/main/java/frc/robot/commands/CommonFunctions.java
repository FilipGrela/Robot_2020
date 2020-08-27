/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class CommonFunctions {
    public final static double eliminateDeadZone(double value, double limit){
        if(Math.abs(value) <= limit ){
          return 0;
        }
        return value;
      } 

    public final static double getRobotWheelsSpeedAverageRPS(){
      return ((getRobotLeftWheelsSpeedRPS()+getRobotRightWheelsSpeedRPS())/2);
    }  

    public final static double getRobotLeftWheelsSpeedRPS(){
      return (-(((double)Robot.driveTrain.leftMaster.getSelectedSensorVelocity()/4096)*10)/3);
    }  

    public final static double getRobotRightWheelsSpeedRPS(){
      return ((((double)Robot.driveTrain.rightMaster.getSelectedSensorVelocity()/4096)*10)/3);
    }  
    
}
