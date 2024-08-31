// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants.SwerveChassis;
import frc.robot.Constants.SwerveConstants.TunerConstants;
import frc.robot.Constants.SwerveConstants.SwerveChassis.SwerveModuleConstantsEnum;

public class DriveSubsystem extends SwerveDrivetrain implements Subsystem {

  private SwerveModuleConstants[] swerveModuleConstants;

  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveChassis.MaxSpeed * 0.1).withRotationalDeadband(SwerveChassis.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(double OdometryUpdateFrequency) {
        super(TunerConstants.DrivetrainConstants, OdometryUpdateFrequency, configureSwerveChassis());
    }

  public DriveSubsystem() {
        super(TunerConstants.DrivetrainConstants, configureSwerveChassis());
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
  }

  public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s) {
    this.setControl(
      drive.withVelocityX(xVelocity_m_per_s)
        .withVelocityY(yVelocity_m_per_s)
        .withRotationalRate(omega_rad_per_s)
    );
  }

    /** 
   * Field Centric Pose of the chassis
   * We get it from odometry, rather than sensors. That means commands that use it must ensure that
   * odometry was properly updated.
  */
  public Pose2d getPose() {
    return this.getState().Pose;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return this.getState().speeds;
  }

  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    drive(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond,
      speeds.omegaRadiansPerSecond
    );
  }


  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? SwerveChassis.redAlliancePerspectiveRotation
                                : SwerveChassis.blueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
  }
}
