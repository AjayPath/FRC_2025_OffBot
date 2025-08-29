// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class IntakeSubystem extends SubsystemBase {
  /** Creates a new IntakeSubystem. */
  private final SparkMax intakeMotor1 = new SparkMax(34, MotorType.kBrushless);
  private final SparkMax intakeMotor2 = new SparkMax(35, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder1 = intakeMotor1.getEncoder();
  private final RelativeEncoder intakeEncoder2 = intakeMotor2.getEncoder();
  
  private static final double INTAKE_SPEED = 1;

  public IntakeSubystem() {

    intakeMotor1.configure(
      Configs.ArmSubsystem.armConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);

    intakeMotor2.configure(
        Configs.ArmSubsystem.armConfig, 
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters);

    intakeEncoder1.setPosition(0);
    intakeEncoder2.setPosition(0);

  }

  public void runIntakeForward() {
    intakeMotor1.set(INTAKE_SPEED);
    intakeMotor2.set(-INTAKE_SPEED);
  }

  public void runIntakeReverse() {
    intakeMotor1.set(-INTAKE_SPEED);
    intakeMotor2.set(INTAKE_SPEED);
  }

  public void stopIntake() {
    intakeMotor1.set(0.05);
    intakeMotor2.set(-0.05);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
