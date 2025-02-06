// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.OutTake;
// import swervelib.encoders.TalonSRXEncoderSwerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoController extends SequentialCommandGroup {
  private static Drive drive;
  private static OutTake outtake;
  
    /** Creates a new Auto. */
    public AutoController(OutTake motor, Drive y) {
      if (motor == null) {
        System.out.println("motor is null");
        // throw new NullPointerException("Outtake Motor is Null in AutoController");
      }
  
      // hi guys
      // outtake = motor;

      TalonSRX x = new TalonSRX(1);
      x.setInverted(false);
      x.configPeakCurrentLimit(40);

      outtake = new OutTake(x);

      drive = y;

      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      // addCommands();
    }
  
    public Command Release() {
      return Commands.sequence(
        Commands.run(
          () -> outtake.setSpeed(-1),
           outtake
         ).raceWith(Commands.waitSeconds(3))

      // Commands.run(
      //   () -> outtake.setSpeed(0),
      //   outtake 
      //   )
      );
  }
  public static Command runMotor(TalonSRX x, double speed) {
    return Commands.sequence(
      Commands.run(
        () -> x.set(TalonSRXControlMode.PercentOutput, 
        speed), 
        outtake
        ).raceWith(Commands.waitSeconds(3)));

  }

  public static Command driveDir(int dir) {
    final double _speed;
    if (dir == 1) {
      _speed = -.3;
    } else {
      _speed = .3;
    }
    return
      Commands.run(() -> drive.driveDir(_speed, _speed), drive)
    ;
  }
}
