// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private final TalonSRX mFR;
  private final TalonSRX mFL;
  private final TalonSRX mBR;
  private final TalonSRX mBL;

  /** Creates a new Drive. */
  public Drive(TalonSRX fR,TalonSRX fL,TalonSRX bR,TalonSRX bL) {
    this.mFR=fR;
    this.mFL=fL;
    this.mBR=bR;
    this.mBL=bL;
  }

  public void driveDir(double y1, double y2) {
    /*
     * y1 is the LEFT JOYSTICK
     * y2 is the RIGHT JOYSTICK
     */

     // inverts because of values funky
     y1 = y1*-1;
     y2=y2*-1; 
     
     mBL.set(TalonSRXControlMode.PercentOutput,y1);
     mBR.set(TalonSRXControlMode.PercentOutput,y2);
     mFL.set(TalonSRXControlMode.PercentOutput,y1);
     mFR.set(TalonSRXControlMode.PercentOutput,y2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
