// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.utils.SimPID;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final SparkFlex elevatorMotor1 = new SparkFlex(50, MotorType.kBrushless);
  private final SparkFlex elevatorMotor2 = new SparkFlex(51, MotorType.kBrushless);

  private final RelativeEncoder elevatorMotor1Encoder = elevatorMotor1.getEncoder();
  private final RelativeEncoder elevatorMotor2Encoder = elevatorMotor2.getEncoder();

  private final SimPID elevatorPID;

  private static final double GEAR_RATIO = 3;

  private double targetPosition = 0;

  public ElevatorSubsystem() {

    elevatorMotor1.configure(
      Configs.ElevatorSubystem.elevatorConfig1, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);

    elevatorMotor2.configure(
      Configs.ElevatorSubystem.elevatorConfig2, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);

    elevatorPID = new SimPID(0.035, 0, 0, 0);
    elevatorPID.setMaxOutput(1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ELE TARGET", targetPosition);
    SmartDashboard.putNumber("ELE ENCODER", elevatorMotor1Encoder.getPosition());
  }

  public void setTargetPosition (double position) {
    targetPosition = position;
    elevatorPID.setDesiredValue(targetPosition);
  }

  public void runPID() {
    double currentPosition = elevatorMotor1Encoder.getPosition();
    double output = elevatorPID.calcPID(currentPosition);

    elevatorMotor1.set(output);
    elevatorMotor2.set(output);
  }

  public boolean atTarget() {
    return elevatorPID.isDone();
  }

  public void stop() {
    elevatorMotor1.stopMotor();
    elevatorMotor2.stopMotor();;
  }

}
