// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTrajectorySequenceRobotAtStartPoint extends SequentialCommandGroup {
  /** Creates a new RunTrajectorySequenceRobotAtStartPoint. */
  public RunTrajectorySequenceRobotAtStartPoint() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }

  public RunTrajectorySequenceRobotAtStartPoint(PathPlannerPath traj, double maxVelocity, double maxAngularVelocity,
      boolean reversed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Read the trajectory from a file
    this.trajectoryPath = traj;

    addCommands(
        // new InstantCommand(RobotContainer.driveSubsystem::zeroDriveEncoders),
        new PrintCommand("****Starting trajectory****"),
        // new WaitCommand(0.4),
        new InstantCommand(() -> RobotContainer.imuSubsystem
            .setYawForTrajectory(trajectoryPath.getInitialHolonomicPose().getRotation().getDegrees())),
        new InstantCommand(() -> RobotContainer.driveSubsystem.resetOdometry(trajectoryPath.getInitialHolonomicPose())),
        // new PrintCommand(
        // "START IX:" + trajectoryPath.getInitialPose().getX()+
        // " IY:" + trajectoryPath.getInitialPose().getY()+
        // " IA:" + trajectoryPath.getInitialPose().getRotation().getDegrees()
        // ), // Set the initial pose of the robot to the one in a trajectory
        new AutonomousTrajectoryRioCommand(trajectoryPath)
        //, // Run a trajectory
        .finallyDo (
          () -> RobotContainer.imuSubsystem.restoreYawAfterTrajectory()
          //new InstantCommand(() -> RobotContainer.imuSubsystem.restoreYawAfterTrajectory())
        ),
        new PrintCommand("****End trajectory****"));
  }
}
