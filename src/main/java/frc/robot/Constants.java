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
import edu.wpi.first.units.Unit;

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
      public static final double steerGainsKD = 0.0;
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
      private static final double kWheelRadiusInches = 2*(5.33/5.71
      );

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
      public static double maxAcceleration = 41.68; //this is Max linear acceleration units: m/s^2
      public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
      public static double maxAngularAcceleration = 37.6992; // this is max angular acceleration units: rad/s^2

      // Customize the following values to your prototype
			public static final double metersPerRotationFX = ( (6.75/6.12)*(107.66/100.0)*(1.0 / 48622.0) ) * 2048.0; // measure this number on the robot - remeasure on carpet
      // drive motor only
      public static final double degreePerRotationFX = (1.0 / 122.11575) * 2048; // Angle motor only
      // On our swerve prototype 1 angular rotation of
      // the wheel = 1 full rotation of the encoder

      /**
       * Drive Motor PID. Assumed to be the same for all drive motors
       * These PID constants are only used for auto trajectory driving, and not
       * teleop.
       * We found that changing them a bit will not have a substantial impact on the
       * trajectory with PathPlanner
       * even if a trajectory includes a holonomic component.
       */
      public static final double DRIVE_CHASSIS_KP = 3.5;
      public static final double DRIVE_CHASSIS_KI = 0.00;
      public static final double DRIVE_CHASSIS_KD = 0.1;

      /**
       * Angle Motor PID. Assumed to be the same for all angle motors
       * These PID constants are only used for auto trajectory driving, and not
       * teleop.
       * Changes to these constants will have a substantial impact on the precision of
       * your
       * trajectory if it includes holonomic rotation.
       * Make sure to test the values and adjust them as needed for your robot.
       */
      public static final double ANGLE_CHASSIS_KP = 6.25;
      public static final double ANGLE_CHASSIS_KI = 0.4;
      public static final double ANGLE_CHASSIS_KD = 0.7;

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
            // -0.296142578125, // angleOffset of cancoder to mark zero-position
            -0.296142578125 + 0.013672, // angleOffset of cancoder to mark zero-position
            false, // Inversion for drive motor
            true // Inversion for angle motor
        ),
        MOD1( // Front Right
            3, // driveMotorID
            4, // angleMotorID
            21, // CanCoder Id
            // 0.041015625, // angleOffset of cancoder to mark zero-position
            0.041015625, //angleOffset of cancoder to mark zero-position
            true, // Inversion for drive motor
            true // Inversion for angle motor
        ),
        MOD2( // Back Left
            5, // driveMotorID
            6, // angleMotorID
            22, // CanCoder Id
            //-0.296142578125, // angleOffset of cancoder to mark zero-position
            0.326171875 - 0.009033, // angleOffset of cancoder to mark zero-position
            false, // Inversion for drive motor
            true // Inversion for angle motor
        ),
        MOD3( // Back Right
            7, // driveMotorID
            8, // angleMotorID
            23, // CanCoder Id
            // 0.326171875, // angleOffset of cancoder to mark zero-position
            0.0576171875, // angleOffset of cancoder to mark zero-position
          // 0.3076171875, // angleOffset of cancoder to mark zero-position
            true, // Inversion for drive motor
            true // Inversion for angle motor
        );

        private int driveMotorID;
        private int angleMotorID;
        private int cancoderID;
        private double angleOffset;
        private boolean driveMotorInverted;
        private boolean angleMotorInverted;


        SwerveModuleConstantsEnum(int d, int a, int c, double o,
            boolean di, boolean ai) {
          this.driveMotorID = d;
          this.angleMotorID = a;
          this.cancoderID = c;
          this.angleOffset = o;
          this.driveMotorInverted = di;
          this.angleMotorInverted = ai;
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

        public boolean isDriveMotorInverted() {
          return driveMotorInverted;
        }

        public boolean isAngleMotorInverted() {
          return angleMotorInverted;
        }

        public int getCancoderID() {
          return cancoderID;
        }

      } // End ENUM SwerveModuleConstants
    }

    public class SysIdConstants {
     public static final double rampRate = 0.01;
     public static final double stepVoltage = 0.05; 
     public static final double timeOut = Units.millisecondsToSeconds(5000);
    }
  }

  public static final class CurrentLimiter {
		public static int drive = 45;
		public static int intake = 0;
		public static int arm = 40;
		public static int shooter = 40;
	}
  public static final class DebugTelemetrySubsystems {
		
		public static final boolean odometry = true;
		public static final boolean imu = true;

		public static final boolean arm = false;
		public static final boolean intake = true;
		public static final boolean shooter = false;
		public static final boolean noteHunting = false;
		public static final boolean llAprilTag = true;
		public static final boolean pvAprilTag = false;

		// Calibration-only methods
		public static final boolean calibrateArm = false;
		public static final boolean calibrateIntake = false;
		public static final boolean calibrateShooter = false;

	}

  public static final class EnableCurrentLimiter {
		public static final boolean drive = true;
		public static final boolean intake = true;
		public static final boolean arm = true;
		public static final boolean shooter = true;
	}

  public static final class EnabledSubsystems {
		public static final boolean arm = true;
		public static final boolean intake = true;
		public static final boolean shooter = true;
		public static final boolean climber = true;
		public static final boolean candle = true;
		public static final boolean driverCamera =  true;
		public static final boolean noteHuntingCamera = true;
		public static final boolean llAprilTagCamera = true;
		public static final boolean pvAprilTagCamera = false;
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
          4, // Port Number for Xbox controller
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

  public static final class GPMConstants {
		public static final class Arm {

			public static enum ArmMotorConstantsEnum {
				LEFTMOTOR( // Front Left - main motor
						32, // CANID
						true, // Inversion
						false // Follower
				),
				RIGHTMOTOR( // Front Left
						31, // CANID
						true, // Inversion
						true // Follower
				);

				private int armMotorID; // CAN ID
				private boolean armMotorInverted;
				private boolean armMotorFollower;

				ArmMotorConstantsEnum(int cid, boolean i, boolean f) {
					this.armMotorID = cid;
					this.armMotorInverted = i;
					this.armMotorFollower = f;
				}

				public int getArmMotorID() {
					return armMotorID;
				}

				public boolean getArmMotorInverted() {
					return armMotorInverted;
				}

				public boolean getArmMotorFollower() {
					return armMotorFollower;
				}
			}

			public static final class ArmPIDConstants {

				public static final double kP = 0.02;
				public static final double kI = 0.000;
				public static final double kD = 2.0;
				public static final double kF = 0;
				public static final double kMaxOutput = 0.6;
				public static final double Acceleration = 6750; // raw sensor units per 100 ms per second
				public static final double CruiseVelocity = 6750; // raw sensor units per 100 ms
				public static final int Smoothing = 3; // CurveStrength. 0 to use Trapezoidal Motion Profile. [1,8] for
														// S-Curve (greater value yields greater smoothing).
				public static final double DefaultAcceptableError = 5; // Sensor units
				public static final double Izone = 500;
				public static final double PeakOutput = 0.5; // Closed Loop peak output
				public static final double NeutralDeadband = 0.001;
				public static final int periodMs = 10; // status frame period
				public static final int timeoutMs = 30; // status frame timeout
				public static final int closedLoopPeriod = 1; // 1ms for TalonSRX and locally connected encoder

				public static final double anglePIDTolerance = 0.5; // degree tolerance when rotating arm to angle using PID

			}

			// Arm IMU
			public static final int PIGEON2_ARM_CAN_ID = 16;
			public static final boolean USE_PAN_IMU_FOR_CORRECTION = true; // Correct Arm IMU with Pan IMU if game surface is uneven
			public static final double ARM_ENCODER_CHANGE_PER_DEGREE = 3.862568732		; //TODO: test and correct as needed

			//TODO: Check conversion factors; find the ones that work best with PID
			public static final double POSITION_CONVERSION_FACTOR = 2*Math.PI;
			public static final double VELOCITY_CONVERSION_FACTOR = 2*Math.PI/60;
			public static final double nominalVoltage = 12.0;
			public static final int shooterMotorCurrentLimit = 40;
			public static final double positionConversionFactor = 0;
			public static final double rampRate = 0.25;

			// TODO: Calibrate all these angles
			public static final double ARM_MIN_ANGLE = -83.0;
			public static final double ARM_MAX_ANGLE = 15.0;
			public static final double ARM_INTAKE_ANGLE = -83.0;
			public static final double ARM_AMP_ANGLE = 15.0;
			public static final double ARM_NOTE_VISION_ANGLE = -69.0;	//BASED ON TESTING MAR 11
			public static final double ARM_NOTE_VISION_ANGLE_FOR_AUTO_NOTE_PICKUP = -64.0;	//BASED ON TESTING MAR 11
			public static final double ARM_CLIMB_ANGLE = 0;	//TODO: test this
			public static final double ARM_IMU_RESET_ANGLE = -82.0;

			public static final double armDownPowerForRecalibration = -0.2;
		}

	}
}