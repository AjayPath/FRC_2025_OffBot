// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.utils.SimPID;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final SparkMax armMotor = new SparkMax(55, MotorType.kBrushless);

  private final RelativeEncoder armEncoder = armMotor.getEncoder();

  private final SimPID armPID;

  private static final double GEAR_RATIO = 70;
  private static final double ENCODER_TO_DEGREES = 360/GEAR_RATIO;

  private double targetPosition = 0;

  public ArmSubsystem() {

    armMotor.configure(
      Configs.ArmSubsystem.armConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);

    armEncoder.setPosition(0);

    armPID = new SimPID(0.025, 0, 0, 1);
    armPID.setMaxOutput(0.7);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TARGET", targetPosition);
    SmartDashboard.putNumber("ARM ENCODER", armEncoder.getPosition() * (360/70));
  }

  public void setTargetPosition (double positionDegrees) {
    targetPosition = positionDegrees;
    armPID.setDesiredValue(positionDegrees);
  }

  public void runPID() {
    double currentPosition = armEncoder.getPosition() * ENCODER_TO_DEGREES;
    double output = armPID.calcPID(currentPosition);

    armMotor.set(output);
  }

  public boolean atTarget() {
    return armPID.isDone();
  }

  public void stop() {
    armMotor.stopMotor();
  }


}
