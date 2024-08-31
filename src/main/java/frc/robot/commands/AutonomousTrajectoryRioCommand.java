// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants.SwerveChassis;

/** Add your docs here. */
public class AutonomousTrajectoryRioCommand extends FollowPathHolonomic{

    public AutonomousTrajectoryRioCommand(PathPlannerPath trajectoryPath) {
        super(
            trajectoryPath,
            RobotContainer.driveSubsystem::getPose,
            RobotContainer.driveSubsystem::getChassisSpeeds,
            RobotContainer.driveSubsystem::driveWithChassisSpeeds,
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(
                        SwerveChassis.DRIVE_CHASSIS_KP,
                        SwerveChassis.DRIVE_CHASSIS_KI,
                        SwerveChassis.DRIVE_CHASSIS_KD), // Translation PID constants
                    new PIDConstants(
                        SwerveChassis.ANGLE_CHASSIS_KP, 
                        SwerveChassis.ANGLE_CHASSIS_KI,
                        SwerveChassis.ANGLE_CHASSIS_KD
                    ), // Rotation PID constants
                    SwerveChassis.MaxSpeed, // Max module speed, in m/s
                    Math.sqrt(Math.pow(SwerveChassis.TRACK_WIDTH/2 ,2) + Math.pow(SwerveChassis.WHEEL_BASE/2 ,2)) , // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
        );
    }

}
