// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import static edu.wpi.first.units.Units.Percent;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoController;
// import frc.robot.commands.AutoController;
import frc.robot.subsystems.OutTake;
// import frc.robot.subsystems.AutoHandler;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color.RGBChannel;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final OutTake mOutTake;
  private final Drive mDrive;
  // private final AutoHandler autos;
  private final AutoController _autos;
  private final LEDs led;
  
  LEDPattern[] colors = {
    LEDPattern.solid(Color.kRed),
    LEDPattern.solid(Color.kGreen),
    LEDPattern.solid(Color.kBlue),
    LEDPattern.solid(Color.kWhite),
    LEDPattern.solid(Color.fromHSV(72, 100, 100))
  };

  private SendableChooser<Command> chooser = new SendableChooser<>();

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
    // Pose2d robotpose = new Pose2d();

    //ALL drive Settings
    //Sets inverted to false
    driveFrontLeft.setInverted(true);
    driveFrontRight.setInverted(true);
    driveBackLeft.setInverted(true);
    driveBackRight.setInverted(true);

    //Sets Peak Amp (Power) limit to 40
    driveFrontLeft.configPeakCurrentLimit(40);
    driveFrontRight.configPeakCurrentLimit(40);
    driveBackLeft.configPeakCurrentLimit(40);
    driveBackRight.configPeakCurrentLimit(40);
    
    LEDPattern red= LEDPattern.solid(Color.kRed);
    AddressableLED _led = new AddressableLED(0);

    led = new LEDs(red, _led);

    // Create mDrive
    mDrive = new Drive(driveFrontRight,driveFrontLeft,driveBackRight,driveBackLeft/*, robotpose*/);

    _autos = new AutoController(mOutTake, mDrive, led);

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

      // AutoCommands();

      Autos();

      SmartDashboard.putData("choices", chooser);

    // Configure the trigger bindings
    configureBindings();
  }

  // private void AutoCommands() {
  //   AutoController auto_ = new AutoController(mOutTake);
  //   NamedCommands.registerCommand("Release", auto_.Release());
    
  //   chooser.setDefaultOption("Default", auto_);
  //   chooser.addOption("Release", auto_.Release());
  // }

  private void Autos() {
    NamedCommands.registerCommand("Spit Coral", _autos.spitCoral());
    chooser.setDefaultOption("Leave Middle 1 point", _autos);
  }

  // public OutTake returnOutTake() {
  //   return mOutTake;
  // }

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

    m_driverController.x().onTrue(Commands.runOnce(() -> led.startColor(), led));
    m_driverController.x().onFalse(Commands.runOnce(() -> led.shutOff(), led));
    // m_driverController.y().onTrue(Commands.runOnce(() -> led.applyGradient(), led));
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
  public Command getAutonomousCommand(String action) {

    // An example command will be run in autonomous
    // return chooser.getSelected();
    // return new PathPlannerAuto("KitBot_TEST");

    // return Commands.print("No autonomous command configured");
    if (action == "Default") {
      return Commands.sequence(
    //   Commands.run(() -> _autos.driveDir(0,3), mDrive).raceWith(Commands.waitSeconds(5)),
    //   Commands.runOnce(() -> _autos.stopDriving(), mDrive).raceWith(Commands.waitSeconds(.2)),
    //   Commands.run(() -> _autos.spitCoral(), mOutTake).raceWith(Commands.waitSeconds(2)),
    //   Commands.run(() -> _autos.driveDir(1,.75), mDrive).raceWith(Commands.waitSeconds(2)),
    //   Commands.runOnce(() -> _autos.stopDriving(), mDrive).raceWith(Commands.waitSeconds(.2))
      _autos.driveDir(0,2.5).raceWith(Commands.waitSeconds(2.5)),
      _autos.stopDriving().raceWith(Commands.waitSeconds(.5)),
      _autos.spitCoral().raceWith(Commands.waitSeconds(1.2)),
      _autos.driveDir(1,1).raceWith(Commands.waitSeconds(1)),
      _autos.stopDriving()

    );
    } else if (action == "Release") {
      return Commands.sequence(
      _autos.driveDir(0,.6).raceWith(Commands.waitSeconds(2.5)),
      _autos.stopDriving().raceWith(Commands.waitSeconds(.5)),
      _autos.spitCoral().raceWith(Commands.waitSeconds(1.2)),
      _autos.driveDir(1,.2).raceWith(Commands.waitSeconds(1)),
      _autos.stopDriving()

    );
    } else {
      return Commands.print("no sigma auto selected");
    }

    // return _autos.spitCoral(); THIS IS ONE COMMAND
  }
}
