// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.AutoHandler;
// import frc.robot.commands.AutoController;
// import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.OutTake;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String _default = "Default";
  private static final String custom = "Release";
  private String autoSel;
  private final SendableChooser<String> chooser = new SendableChooser<>();

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  // private final OutTake outtake;
  // private final TalonSRX motor;
  // private final Drive drive;
  // private final AutoHandler autos;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    chooser.setDefaultOption("Default", _default);
    chooser.addOption("Release", custom);
    SmartDashboard.putData("choices", chooser);

    // TalonSRX x = new TalonSRX(1);
    // x.setInverted(false);
    // x.configPeakCurrentLimit(40);
    // TalonSRX driveFrontRight = new TalonSRX(15);
    // TalonSRX driveFrontLeft = new TalonSRX(2);
    // TalonSRX driveBackRight = new TalonSRX(14);
    // TalonSRX driveBackLeft = new TalonSRX(0);
    // // Pose2d robotpose = new Pose2d();

    // //ALL drive Settings
    // //Sets inverted to false
    // driveFrontLeft.setInverted(false);
    // driveFrontRight.setInverted(false);
    // driveBackLeft.setInverted(false);
    // driveBackRight.setInverted(false);

    // //Sets Peak Amp (Power) limit to 40
    // driveFrontLeft.configPeakCurrentLimit(40);
    // driveFrontRight.configPeakCurrentLimit(40);
    // driveBackLeft.configPeakCurrentLimit(40);
    // driveBackRight.configPeakCurrentLimit(40);

    // // motor = x;
    // outtake = new OutTake(x);
    // drive = new Drive(driveFrontLeft,driveFrontRight,driveBackRight,driveBackLeft);
    // autos = new AutoHandler(outtake, drive);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    autoSel = chooser.getSelected();

    System.out.println("auto selected: " + autoSel);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (autoSel) {
    //   case custom: 
      // AutoController.driveDir(0);
      // autos.driveDir(0, .3, 7.0);
      // autos._wait(8);
      // autos.spitCoral();
      // AutoController.runMotor(motor, 1);

      // motor.set(TalonSRXControlMode.PercentOutput, .2);
    //   System.out.println("THIS IS THE CUSTOM AUTO");
    //   // outtake.setSpeed(.2);
    //   ;
    //   case _default: default: 
    //   motor.set(TalonSRXControlMode.PercentOutput, -.1);
    //   System.out.println("THIS IS THE DEFAULT AUTO");
    //   // outtake.setSpeed(-.1);
    //   ;
    // } 
    System.out.println(autoSel);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
