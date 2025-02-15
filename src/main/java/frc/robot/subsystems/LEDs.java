// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

// import org.w3c.dom.css.RGBColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color.RGBChannel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {
  private final LEDPattern example;
  private final AddressableLED led;

  Color.RGBChannel SOTA_red;
  Color.RGBChannel SOTA_green;
  Color.RGBChannel SOTA_blue;

  LEDPattern blink;
  LEDPattern SOTADemon = LEDPattern.gradient(GradientType.kContinuous, Color.kYellowGreen, Color.kSkyBlue); // sotabots + gradledemons
  // LEDPattern synced = example.blink(RobotContainer::getRSLState);
  LEDPattern rainbow = LEDPattern.rainbow(255, 180);
  LEDPattern noColor;
  AddressableLEDBuffer buffer = new AddressableLEDBuffer(144);
  /** Creates a new LEDs. */
  public LEDs(LEDPattern a, AddressableLED b) {
    example = a;
    led = b;

    // for (var i=0; i < buffer.getLength(); i++) {
    //   buffer.setRGB(i, 204, 254, 0);
    // }

    // example = colors[3];
    
    led.setLength(buffer.getLength());
    noColor = LEDPattern.kOff;
    blink = example.blink(Seconds.of(2));
    // blink.applyTo(buffer);
    // rainbow.applyTo(buffer);
  }

  public void startColor() {
    example.applyTo(buffer);

    led.setData(buffer);
    led.start();
  }

  public void shutOff() {
    led.stop();
    noColor.applyTo(buffer);
    led.setData(buffer);
  }

  public void startBlink() {
    blink.applyTo(buffer);
    led.setData(buffer);
  }

  // public void applyGradient() {
  //   SOTADemon.applyTo(buffer);
  //   led.setData(buffer);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
