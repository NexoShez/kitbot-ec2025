// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
// import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.OutTake;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final OutTake mOutTake;
  private final Drive mDrive;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    TalonSRX outTakeMotor = new TalonSRX(1);
    outTakeMotor.setInverted(false);
    outTakeMotor.configPeakCurrentLimit(40);
    mOutTake = new OutTake(outTakeMotor);

    TalonSRX driveFrontRight = new TalonSRX(15);
    TalonSRX driveFrontLeft = new TalonSRX(2);
    TalonSRX driveBackRight = new TalonSRX(14);
    TalonSRX driveBackLeft = new TalonSRX(0);

    //ALL drive Settings
    //Sets inverted to false
    driveFrontLeft.setInverted(false);
    driveFrontRight.setInverted(false);
    driveBackLeft.setInverted(false);
    driveBackRight.setInverted(false);

    //Sets Peak Amp (Power) limit to 40
    driveFrontLeft.configPeakCurrentLimit(40);
    driveFrontRight.configPeakCurrentLimit(40);
    driveBackLeft.configPeakCurrentLimit(40);
    driveBackRight.configPeakCurrentLimit(40);

    // Create mDrive
    mDrive = new Drive(driveFrontRight,driveFrontLeft,driveBackRight,driveBackLeft);

    // this allows the joysticks to control the 4 drive motors
    // without being a simple "true-false" statement
    mDrive.setDefaultCommand(
      new RunCommand(
        () -> mDrive.driveDir(
        m_driverController.getLeftY(), // basically saying the position of the Y axis on the controller
        m_driverController.getRightY() // will be the power demand given to the motor(s)
        ),
        mDrive
        )
      );

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_driverController.b().onTrue(Commands.runOnce(() -> mDrive.invertControls(), mDrive));

    m_driverController.leftTrigger().onTrue(Commands.run(() -> mOutTake.setSpeed(1), mOutTake));
    m_driverController.leftTrigger().onFalse(Commands.runOnce(() -> mOutTake.setSpeed(0), mOutTake));
    
    m_driverController.rightTrigger().onTrue(Commands.run(() -> mOutTake.setSpeed(-1), mOutTake));
    m_driverController.rightTrigger().onFalse(Commands.runOnce(() -> mOutTake.setSpeed(0), mOutTake));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(null);
  }
}
