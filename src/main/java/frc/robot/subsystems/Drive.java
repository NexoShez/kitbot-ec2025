// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPLTVController;
// import com.pathplanner.lib.controllers.PathFollowingController;
// import com.pathplanner.lib.util.DriveFeedforwards;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.math.kinematics.Odometry;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private final TalonSRX mFR;
  private final TalonSRX mFL;
  private final TalonSRX mBR;
  private final TalonSRX mBL;

  // PATHPLANNER DOESNT WORK FOR DIFFERENTIAL DRIVE BRO
  // private final Pose2d robotpose;
  // private final Rotation2d gyro;
  // DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
  //   gyro.getRotations(),
  //   m_leftEncoder.getDistance(), m_rightEncoder.getDistance(),
  //   new Pose2d(5.0, 13.5, new Rotation2d()));

  private boolean isDriveControlsInv = false;
  // private double deadzone = .2;

  /** Creates a new Drive. */
  public Drive(TalonSRX fR, TalonSRX fL, TalonSRX bR, TalonSRX bL /*Pose2d pose*/) {
    this.mFR = fR;
    this.mFL = fL;
    this.mBR = bR;
    this.mBL = bL;
    // robotpose = pose;
  }

  public void driveDir(double y1, double y2) {
    /*
     * y1 is the LEFT JOYSTICK
     * y2 is the RIGHT JOYSTICK
     */

    // ben wanted this
    if (isDriveControlsInv == true) {
      y1 = y1 * -.5;
      y2 = y2 * .5; // invert by opposite for some reason
    } else {
      y1 = y1 * .5;
      y2 = y2 * -.5;
    }

    // if (y1 < .2) {
    // y1=0;
    // }
    // if (y2 < .2) {
    // y2=0;
    // }

    // if (y1 > .2) {
    mBL.set(TalonSRXControlMode.PercentOutput, y1);
    mFL.set(TalonSRXControlMode.PercentOutput, y1);
    // }

    // if (y2 > .2) {
    mFR.set(TalonSRXControlMode.PercentOutput, y2);
    mBR.set(TalonSRXControlMode.PercentOutput, y2);
    // }
  }
  public void driveRobotRelative(ChassisSpeeds speeds){
    this.driveDir(speeds.vyMetersPerSecond, speeds.vyMetersPerSecond);
    // this.driveDir(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false,false);
  }

  public double getSpeed(String x) {
    if (x=="l" || x=="L") {
      return mFL.getMotorOutputPercent();
    } else if (x=="r" || x=="R") {
      return mFR.getMotorOutputPercent();
    } else {
      return 0;
    }
  }

  public void invertControls() {
    if (isDriveControlsInv == true) {
      isDriveControlsInv = false;
    } else {
      isDriveControlsInv = true;
    }
  }

  // more pathplanenr stuff ew

  // // Creating my kinematics object: track width of 27 inches
  // DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27.0));
  // // Example differential drive wheel speeds: 2 meters per second
  // // for the left side, 3 meters per second for the right side.
  // DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(2.0, 3.0);
  // // Convert to chassis speeds.
  // ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
  // // Linear velocity
  // double linearVelocity = chassisSpeeds.vxMetersPerSecond;
  // // Angular velocity
  // double angularVelocity = chassisSpeeds.omegaRadiansPerSecond;

  // public Pose2d getPose() {
  //   return odo.getPoseMeters();
  // }

  // @SuppressWarnings("unchecked")
  // public void resetPose(Pose2d x) {
  //   odo.resetPosition(robotpose.getRotation(), config, x);
  // }

  // public ChassisSpeeds getRobotRelativeSpeeds() {
  //   return chassisSpeeds;
  // }

  // // // Load the RobotConfig from the GUI settings. You should probably
  // // // store this in your Constants file

  // RobotConfig config;
  // {
  //   try {
  //     config = RobotConfig.fromGUISettings();
  //   } catch (Exception e) {
  //     // Handle exception as needed
  //     e.printStackTrace();
  //   }

  //   // Configure AutoBuilder last
  //   AutoBuilder.configure(
  //       this::getPose, // Robot pose supplier
  //       this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
  //       this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  //       (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
  //                                                             // ChassisSpeeds. Also optionally outputs individual
  //                                                             // module feedforwards
  //       new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential
  //                                  // drive trains
  //       config, // The robot configuration
  //       () -> {
  //         // Boolean supplier that controls when the path will be mirrored for the red
  //         // alliance
  //         // This will flip the path being followed to the red side of the field.
  //         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

  //         var alliance = DriverStation.getAlliance();
  //         if (alliance.isPresent()) {
  //           return alliance.get() == DriverStation.Alliance.Red;
  //         }
  //         return false;
  //       },
  //       this // Reference to this subsystem to set requirements
  //   );
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
