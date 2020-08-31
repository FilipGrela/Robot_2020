/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.debug;

import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DashboardLogs {
    public void logs(){
        Robot.driveTrain.logsDriveTrain();
        Robot.oi.logsOi();
    }
}
