/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Flags;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.commands.Intake.IntakeMove;

public class Intake extends SubsystemBase {

  WPI_VictorSPX intakeMotor;

  public DoubleSolenoid doubleSolenoid;
  Compressor compressor;

  public Intake() {
    doubleSolenoid = new DoubleSolenoid(PortMap.kIntakeSolenoidA, PortMap.kIntakeSolenoidB);
    compressor = new Compressor();
    //compressor.stop();

    intakeMotor = new WPI_VictorSPX(PortMap.kIntakeMotor);
  }

  @Override
  public void periodic() {
    
  }

  public void intakeMoveDashboard(){
    SmartDashboard.putBoolean("Intake State Bool", Flags.getFlags().intakeDown);
  }

  public void intakeUp() {
    doubleSolenoid.set(Value.kForward);
  }

  public void intakeDown() {
    doubleSolenoid.set(Value.kReverse);
  }

  public void intakeOff() {
    doubleSolenoid.set(Value.kOff);
  }

  public void intakeSpeenBackward(){
    intakeMotor.set(ControlMode.PercentOutput, -0.3);
  }

  public void intakeSpeenForward(){
    intakeMotor.set(ControlMode.PercentOutput, 0.3);
  }

  public void intakeMotorStop(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void initIntake(){
    intakeDown();
    try{
    Thread.sleep(1000);
    }catch(InterruptedException ex){
    Thread.currentThread().interrupt();
    }
    intakeOff();
  }
}
