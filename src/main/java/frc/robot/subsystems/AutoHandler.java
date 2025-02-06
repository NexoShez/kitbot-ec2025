// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoHandler extends SubsystemBase {
  private final Drive drive;
  private final OutTake outtake;

  WaitCommand wait_;

  /** Creates a new AutoHandler. */
  public AutoHandler(OutTake _outtake, Drive _drive) {
    if (_outtake==null) {
      throw new NullPointerException("OutTake in AutoHandler.java:14 is null");
    }
    if (_drive==null) {
      throw new NullPointerException("Drive in AutoHandler.java:14 is null");
    }

    outtake=_outtake;
    drive=_drive;

  }

  public void _wait(double x) {
    wait_ = new WaitCommand(x);
  }

  public void driveDir(int dir, double speed, double y) {
    /*
     * 0 = forward (default)
     * 1 = backwards
     */

    if (dir==0) {
      drive.driveDir(-speed, speed);
    } else {
      drive.driveDir(speed, -speed);
    }
    _wait(y);
    drive.driveDir(0, 0);
  }

  public void spitCoral() {
    outtake.setSpeed(1);
    _wait(3);
    outtake.setSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
