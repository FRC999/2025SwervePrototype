// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {
    public static class TunerConstants {
      public static final double steerGainsKP = 100;
      public static final double steerGainsKI = 0;
      public static final double steerGainsKD = 0.2;
      public static final double steerGainsKS = 0;
      public static final double steerGainsKV = 1.5;
      public static final double steerGainsKA = 0;

      private static final Slot0Configs steerGains = new Slot0Configs()
          .withKP(steerGainsKP).withKI(steerGainsKI).withKD(steerGainsKD)
          .withKS(steerGainsKS).withKV(steerGainsKV).withKA(steerGainsKA);

      public static final double driveGainsKP = 3;
      public static final double driveGainsKI = 0;
      public static final double driveGainsKD = 0;
      public static final double driveGainsKS = 0;
      public static final double driveGainsKV = 0;
      public static final double driveGainsKA = 0;

      private static final Slot0Configs driveGains = new Slot0Configs()
          .withKP(driveGainsKP).withKI(driveGainsKI).withKD(driveGainsKD)
          .withKS(driveGainsKS).withKV(driveGainsKV).withKA(driveGainsKA);

      private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
      private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

      private static final double kSlipCurrentA = 150.0;

      private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
      private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a
                  // relatively low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true));

      private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();

      // Theoretical free speed (m/s) at 12v applied output;
      // This needs to be tuned to your individual robot
      public static final double kSpeedAt12VoltsMps = 5.21;

      // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
      // This may need to be tuned to your individual robot
      private static final double kCoupleRatio = 3.5714285714285716;

      private static final double kDriveGearRatio = 6.122448979591837;
      private static final double kSteerGearRatio = 21.428571428571427;
      private static final double kWheelRadiusInches = 2;

      private static final boolean kInvertLeftSide = false;
      private static final boolean kInvertRightSide = true;

      private static final String kCANbusName = "";

      // These are only used for simulation
      private static final double kSteerInertia = 0.00001;
      private static final double kDriveInertia = 0.001;
      // Simulated voltage necessary to overcome friction
      private static final double kSteerFrictionVoltage = 0.25;
      private static final double kDriveFrictionVoltage = 0.25;

      public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
          .withCANbusName(kCANbusName)
          .withPigeon2Id(IMUConstants.kPigeonId)
          .withPigeon2Configs(IMUConstants.pigeonConfigs);

      public static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadiusInches)
          .withSlipCurrent(kSlipCurrentA)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(kCoupleRatio)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withCANcoderInitialConfigs(cancoderInitialConfigs);

    }

    public static class SwerveChassis {

      public static final double TRACK_WIDTH = 0.525; // left to right
      public static final double WHEEL_BASE = 0.525; // front to back
      public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
      public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

      public static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
      public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

      public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

      /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
      public static final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
      /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
      public static final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

      public static enum SwerveModuleConstantsEnum {
        MOD0( // Front Left,
            1, // driveMotorID
            2, // angleMotorID
            20, // CanCoder Id
            -0.296142578125, // angleOffset of cancoder to mark zero-position
            true // Inversion for angle motor
        ),
        MOD1( // Front Right
            3, // driveMotorID
            4, // angleMotorID
            21, // CanCoder Id
            0.041015625, // angleOffset of cancoder to mark zero-position
            true // Inversion for angle motor

        ),
        MOD2( // Back Left
            5, // driveMotorID
            6, // angleMotorID
            22, // CanCoder Id
            -0.296142578125, // angleOffset of cancoder to mark zero-position
            true // Inversion for angle motor

        ),
        MOD3( // Back Right
            1, // driveMotorID
            2, // angleMotorID
            20, // CanCoder Id
            0.326171875, // angleOffset of cancoder to mark zero-position
            true // Inversion for angle motor
        );

        private int driveMotorID;
        private int angleMotorID;
        private double angleOffset;
        private boolean angleMotorInverted;
        private int cancoderID;

        SwerveModuleConstantsEnum(int d, int a, int c, double o,
            boolean ai) {
          this.driveMotorID = d;
          this.angleMotorID = a;
          this.angleOffset = o;
          this.angleMotorInverted = ai;
          this.cancoderID = c;
        }

        public int getDriveMotorID() {
          return driveMotorID;
        }

        public int getAngleMotorID() {
          return angleMotorID;
        }

        public double getAngleOffset() {
          return angleOffset;
        }

        public boolean isAngleMotorInverted() {
          return angleMotorInverted;
        }

        public int getCancoderID() {
          return cancoderID;
        }

      } // End ENUM SwerveModuleConstants
    }
  }

  	/**
	 * Controller-related constants.
	 * Here we define port numbers, axis, deadbands, button numbers and various
	 * ability flags, such as use of the cube driving
	 */
	public static final class OIConstants {
		public static final int driverControllerPort = 0;

		public static final int bblPort = 4;
		public static final int bbrPort = 3;

		public static final int driverInterfaceSwitchButton = 1;

		public static final int robotCentricButton = 5; // XBOX L1 button

		public static final ControllerDeviceType driverInterfaceType = ControllerDeviceType.XBOX_ONEDRIVE;

		public static final int CALIBRATION_JOYSTICK_SLIDER_AXLE = 3;

		public static enum ControllerDeviceType {
			LOGITECH,
			PS5,
			XBOX, // RightJ F/B, LeftJ L/R, L2/R2 - rotation
			XBOX_ONEDRIVE // RIghtJ F/B/L/R, LeftJ - rotation
		}

		public static enum ControllerDevice {
			DRIVESTICK(
					0, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			// DRIVESTICK1,2,3 are used only for GPM calibration
			DRIVESTICK1(
					1, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			DRIVESTICK2(
					2, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			DRIVESTICK3(
					3, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			TURNSTICK( // Controls the rotation of the swervebot
					2, // Port Number
					ControllerDeviceType.LOGITECH,
					0.02, // deadband X
					0.02, // deadband Y
					0.02, // deadband Omega
					true, // cubeControllerLeft
					true // cubeControllerRight
			),

			XBOX_CONTROLLER(
					5, // Port Number for Xbox controller
					ControllerDeviceType.XBOX,
					0.03, // deadband X for Xbox
					0.03, // deadband Y for Xbox //TODO: ALL DEADBAND FOR XBOX IS PLACEHOLDER
					0.03, // deadband Omega for Xbox
					false, // No cube controller configuration for Xbox yet
					false),

			XBOX_CONTROLLER_GPM(
					4,  // Port Number for Xbox controller
					ControllerDeviceType.XBOX,
					0.03, // deadband X for Xbox
					0.03, // deadband Y for Xbox //TODO: ALL DEADBAND FOR XBOX IS PLACEHOLDER
					0.03, // deadband Omega for Xbox
					false, // No cube controller configuration for Xbox yet
					false);


			private ControllerDeviceType controllerDeviceType;
			private int portNumber;
			private double deadbandX;
			private double deadbandY;
			private double deadbandOmega;
			private boolean cubeControllerLeftStick;
			private boolean cubeControllerRightStick;

			ControllerDevice(int pn, ControllerDeviceType cdt, double dx, double dy, double dm, boolean ccL,
					boolean ccR) {
				this.portNumber = pn;
				this.controllerDeviceType = cdt;
				this.deadbandX = dx;
				this.deadbandY = dy;
				this.deadbandOmega = dm;
				this.cubeControllerLeftStick = ccL;
				this.cubeControllerRightStick = ccR;
			}

			public ControllerDeviceType getControllerDeviceType() {
				return controllerDeviceType;
			}

			public int getPortNumber() {
				return portNumber;
			}

			public double getDeadbandX() {
				return deadbandX;
			}

			public double getDeadbandY() {
				return deadbandY;
			}

			public double getDeadbandOmega() {
				return deadbandOmega;
			}

			public boolean isCubeControllerLeftStick() {
				return cubeControllerLeftStick;
			}

			public boolean isCubeControllerRightStick() {
				return cubeControllerRightStick;
			}
		}
	}

  public static class IMUConstants {
    public static final int kPigeonId = 15;

    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;
  }
}
