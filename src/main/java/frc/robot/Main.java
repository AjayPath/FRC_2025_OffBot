// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.utils.Vector;
import frc.robot.utils.SimPID;
import frc.robot.utils.Pose;
import frc.robot.utils.Calculations;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);

    // System.out.println("Testing Utilities");

    // Vector v1 = new Vector(3, 4);
    // System.out.println("Vector Magnitude: " + v1.GetMag());

    // Vector v2 = new Vector(1, 2);
    // Vector sum = v1.AddVector(v2);
    // System.out.println("Sum of v1 and v2: " + sum.GetX() + ", " + sum.GetY());

    // SimPID pid = new SimPID(1.0, 0, 0.1, 0);
    // pid.setDesiredValue(10);
    // System.out.println("PID output: " + pid.calcPID(2));

  }
}
