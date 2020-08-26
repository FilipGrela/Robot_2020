/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
/**
 * Add your docs here.
 */
public class OI {

    public Joystick driver;
    public Joystick operator;

    public OI(){
        
        driver = new Joystick(0);
        operator = new Joystick(1);

    }

    public  Joystick getDriverJoystick(){
        return driver;
    }

    public  Joystick getOperatorJoystick(){
        return operator;
    }

}
