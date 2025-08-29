// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Vector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VectorTestCommand extends Command {
  /** Creates a new VectorTestCommand. */
  public VectorTestCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Vector v1 = new Vector(3, 4);
    Vector v2 = new Vector(7, 9);
    SmartDashboard.putNumber("Vector X: ", v1.GetXValue());
    SmartDashboard.putNumber("Vector Y: ", v1.GetYValue());
    SmartDashboard.putNumber("Vector2 X: ", v2.GetXValue());
    SmartDashboard.putNumber("Vector2 Y: ", v2.GetYValue());

    Vector v3 = v1.SubtractVector(v2);
    SmartDashboard.putNumber("New Vector X: ", v3.GetXValue());
    SmartDashboard.putNumber("New Vector Y: ", v3.GetYValue());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
