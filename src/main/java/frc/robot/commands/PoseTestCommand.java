// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Pose;
import frc.robot.utils.Vector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoseTestCommand extends Command {
  /** Creates a new PoseTestCommand. */
  private Pose pose1;
  private Pose pose2;

  public PoseTestCommand() {
    // Use addRequirements() here to declare subsystem dependencies.

    pose1 = new Pose(10, 20, Rotation2d.fromDegrees(45));
    pose2 = new Pose(5, 15, Rotation2d.fromDegrees(30));

    SmartDashboard.putNumber("_X 1", pose1.GetXValue());
    SmartDashboard.putNumber("_Y 1", pose1.GetYValue());
    SmartDashboard.putNumber("_Angle 1: ", pose1.GetAngleValue());

    SmartDashboard.putNumber("_X 2", pose2.GetXValue());
    SmartDashboard.putNumber("_Y 2", pose2.GetYValue());
    SmartDashboard.putNumber("_Angle 2: ", pose2.GetAngleValue());

    Vector difference = pose1.Subtract(pose2);
    SmartDashboard.putNumber("X: ", difference.GetXValue());
    SmartDashboard.putNumber("Y: ", difference.GetYValue());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
