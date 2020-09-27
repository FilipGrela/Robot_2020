/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Intake.IntakeMove;
import frc.robot.commands.Intake.IntakeSpinBackward;
import frc.robot.commands.Intake.IntakeSpinForward;
/**
 * Add your docs here.
 */
public class OI {

    public Joystick driver;
    public Joystick operator;

    public JoystickButton buttonIntakeMove;
    public JoystickButton buttonIntakeForward;
    public JoystickButton buttonIntakeBackward;

    public OI(){
        driver = new Joystick(0);
        operator = new Joystick(1);

        buttonIntakeMove = new JoystickButton(driver, 1); //kwadrat ps4
        buttonIntakeMove.whileHeld(new IntakeMove());

        buttonIntakeForward = new JoystickButton(driver, 5); //L1 ps4
        buttonIntakeForward.whileHeld(new IntakeSpinForward());
        buttonIntakeBackward = new JoystickButton(driver, 6); //R1 ps4
        buttonIntakeBackward.whileHeld(new IntakeSpinBackward());
    }

    public  Joystick getDriverJoystick(){
        return driver;
    }

    public  Joystick getOperatorJoystick(){
        return operator;
    }

    public void logsOi(){
        SmartDashboard.putNumber("JoyStick_Speed_Axis", getDriverJoystick().getRawAxis(PortMap.kSpeedAxisPort));
        SmartDashboard.putNumber("JoyStick_Turn_Axis", getDriverJoystick().getRawAxis(PortMap.kTurnAxisPort));
    }
}
