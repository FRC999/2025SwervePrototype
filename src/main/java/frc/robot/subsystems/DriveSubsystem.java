// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants.SwerveChassis;
import frc.robot.Constants.SwerveConstants.TunerConstants;
import frc.robot.Constants.SwerveConstants.SwerveChassis.SwerveModuleConstantsEnum;

public class DriveSubsystem extends SwerveDrivetrain implements Subsystem {

  private SwerveModuleConstants[] swerveModuleConstants;
  private CommandSwerveDrivetrain driveChassis;

  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency) {
        super(driveTrainConstants, OdometryUpdateFrequency, configureSwerveChassis());
    }

  public DriveSubsystem(SwerveDrivetrainConstants driveTrainConstants) {
        super(driveTrainConstants, configureSwerveChassis());
    }

  public static SwerveModuleConstants[] configureSwerveChassis() {
    return new SwerveModuleConstants[] {
        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD0.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD0.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD0.getCancoderID(),
            SwerveModuleConstantsEnum.MOD0.getAngleOffset(),
            SwerveChassis.WHEEL_BASE / 2.0,
            SwerveChassis.TRACK_WIDTH / 2.0,
            SwerveModuleConstantsEnum.MOD0.isAngleMotorInverted()),

        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD1.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD1.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD1.getCancoderID(),
            SwerveModuleConstantsEnum.MOD1.getAngleOffset(),
            SwerveChassis.WHEEL_BASE / 2.0,
            -SwerveChassis.TRACK_WIDTH / 2.0,
            SwerveModuleConstantsEnum.MOD1.isAngleMotorInverted()),

        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD2.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD2.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD2.getCancoderID(),
            SwerveModuleConstantsEnum.MOD2.getAngleOffset(),
            -SwerveChassis.WHEEL_BASE / 2.0,
            SwerveChassis.TRACK_WIDTH / 2.0,
            SwerveModuleConstantsEnum.MOD2.isAngleMotorInverted()),

        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD3.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD3.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD3.getCancoderID(),
            SwerveModuleConstantsEnum.MOD3.getAngleOffset(),
            -SwerveChassis.WHEEL_BASE / 2.0,
            -SwerveChassis.TRACK_WIDTH / 2.0,
            SwerveModuleConstantsEnum.MOD3.isAngleMotorInverted())
    };

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
        FrontRight, BackLeft, BackRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
