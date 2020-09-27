/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Flags {
    private static Flags instance;
	
	public static Flags getFlags() {		
		if ( instance == null ) {
				instance = new Flags();
		}
		return instance;
    }

    public boolean intakeDown = true;
}
