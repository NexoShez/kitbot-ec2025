// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OutTake extends SubsystemBase {
  /** Creates a new OutTa ke. */

  private final TalonSRX motor;

  public OutTake(TalonSRX motor) {
    this.motor = motor;
  }

  public void setSpeed(double speed) {
    motor.set(TalonSRXControlMode.PercentOutput,speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
