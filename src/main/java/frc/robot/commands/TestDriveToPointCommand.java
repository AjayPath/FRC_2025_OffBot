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
public class TestDriveToPointCommand extends Command {
  /** Creates a new TestDriveToPointCommand. */
  private Pose currentPose;
  private Pose targetPose;

  public TestDriveToPointCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPose = new Pose (0, 0, Rotation2d.fromDegrees(0));
    targetPose = new Pose(10, 10, Rotation2d.fromDegrees(0));
    updateDashboard();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Vector errorVector = targetPose.Subtract(currentPose);
    double distance = Math.sqrt(errorVector.GetXValue() * errorVector.GetXValue() + errorVector.GetYValue() * errorVector.GetYValue());
    SmartDashboard.putNumber("Distance to Target", distance);

    if (distance > 0.5) {
      double unitX = errorVector.GetXValue() / distance;
      double unitY = errorVector.GetYValue() / distance;

      double moveX = unitX * 0.05;
      double moveY = unitY * 0.05;

      // Update current pose
      currentPose.SetX(currentPose.GetXValue() + moveX);
      currentPose.SetY(currentPose.GetYValue() + moveY);

      // Keep angle at zero
      currentPose.SetAngle(Rotation2d.fromDegrees(0));

      updateDashboard();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Vector errorVector = targetPose.Subtract(currentPose);
    double distance = Math.sqrt(errorVector.GetXValue() * errorVector.GetXValue() + errorVector.GetYValue() * errorVector.GetYValue());
    return distance <= 0.5;
  }

  private void updateDashboard() {
    SmartDashboard.putNumber("Current x", currentPose.GetXValue());
    SmartDashboard.putNumber("Current Y", currentPose.GetYValue());
    SmartDashboard.putNumber("Current Anlgle", currentPose.GetAngleValue());
    SmartDashboard.putNumber("Target X", targetPose.GetXValue());
    SmartDashboard.putNumber("Target Y", targetPose.GetYValue());
    SmartDashboard.putNumber("Target Angle", targetPose.GetAngleValue());
  }

}
