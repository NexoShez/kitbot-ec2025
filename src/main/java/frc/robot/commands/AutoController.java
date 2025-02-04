// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.Drive;
import frc.robot.subsystems.OutTake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoController extends SequentialCommandGroup {
  // private final Drive drive;
  private final OutTake outtake;

  /** Creates a new Auto. */
  public AutoController(OutTake motor) {
    if (motor == null) {
      throw new NullPointerException("Outtake Motor is Null in AutoController");
    }

    // hi guys
    outtake = motor;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands();
  }

  public Command Release() {
    return Commands.sequence(
      Commands.run(
        () -> outtake.setSpeed(-1),
         outtake
         ).raceWith(Commands.waitSeconds(3)),

      Commands.runOnce(
        () -> outtake.setSpeed(0),
        outtake 
        )
      );
  }
}
