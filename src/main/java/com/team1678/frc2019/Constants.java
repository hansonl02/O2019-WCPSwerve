package com.team1678.frc2019;

import java.util.Arrays;
import java.util.List;

import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class Constants {
	/*All distance measurements are in inches, unless otherwise noted.*/

	public static final double kLooperDt = 0.02;
	
	public static final double kEpsilon = 0.0001;
	
	public static final boolean kIsUsingCompBot = true;
	public static final boolean kIsUsingTractionWheels = true;

	public static final boolean kDebuggingOutput = false;
	
	//Physical Robot Dimensions (including bumpers)
	public static final double kRobotWidth = 36.5;
	public static final double kRobotLength = 36.5;
	public static final double kRobotHalfWidth = kRobotWidth / 2.0;
	public static final double kRobotHalfLength = kRobotLength / 2.0;
	public static final double kRobotProbeExtrusion = 4.0;

	public static final double kBallRadius = 6.5;
	
	//Field Landmarks
	public static final Pose2d closeHatchPosition = new Pose2d(new Translation2d(48.0 + 166.57, 27.44 - 10.0 - 162.0), Rotation2d.fromDegrees(-30.0));
	public static final Pose2d rightCloseHatchPosition = new Pose2d(new Translation2d(closeHatchPosition.getTranslation().x(), -closeHatchPosition.getTranslation().y()), Rotation2d.fromDegrees(30.0));
	public static final Pose2d farHatchPosition = new Pose2d(new Translation2d(229.13 + 14.71, 27.44 - 10.0 - 162.0), Rotation2d.fromDegrees(-150.0));
	public static final Pose2d humanLoaderPosition = new Pose2d(new Translation2d(0.0, 25.72 - 162.0), Rotation2d.fromDegrees(0.0));
	public static final Pose2d autoBallPosition = new Pose2d(new Translation2d(48.0 - 4.0 - kBallRadius, 97.0 - (3.0*kBallRadius) - 162.0), Rotation2d.fromDegrees(-45.0));
	public static final Pose2d rocketPortPosition = new Pose2d(new Translation2d(229.13, 27.44 - 162.0), Rotation2d.fromDegrees(-90.0));

	public static final Pose2d closeShipPosition = new Pose2d(new Translation2d(260.8, -28.87), Rotation2d.fromDegrees(90.0));
	public static final Pose2d midShipPosition = new Pose2d(new Translation2d(282.55 + 1.0, -28.87), Rotation2d.fromDegrees(90.0));
	public static final Pose2d farShipPosition = new Pose2d(new Translation2d(304.3, -28.87), Rotation2d.fromDegrees(90.0));

	public static final double kDiskTargetHeight = 28.625;//28.1875
	public static final double kBallTargetHeight = 36.9375;
	public static final List<Rotation2d> kPossibleDiskTargetAngles = Arrays.asList(/*Rotation2d.fromDegrees(0.0),*/ Rotation2d.fromDegrees(30.0),
		Rotation2d.fromDegrees(150.0), /*Rotation2d.fromDegrees(180.0),*/ Rotation2d.fromDegrees(-150.0),
		Rotation2d.fromDegrees(-30.0));
	public static final List<Rotation2d> kPossibleBallTargetAngles = Arrays.asList(Rotation2d.fromDegrees(90.0),
		Rotation2d.fromDegrees(-90.0));
	
	public static final Pose2d kRobotLeftStartingPose = new Pose2d(new Translation2d(48.0 + kRobotHalfLength, 97.0 + kRobotHalfWidth - 162.0), Rotation2d.fromDegrees(0));
	public static final Pose2d kRobotRightStartingPose = new Pose2d(new Translation2d(48.0 + kRobotHalfLength, -(97.0 + kRobotHalfWidth - 162.0)), Rotation2d.fromDegrees(0));
	public static final Pose2d kRobotLeftRampExitPose = new Pose2d(new Translation2d(95.25 + kRobotHalfLength, 97.0 + kRobotHalfWidth - 162.0), Rotation2d.fromDegrees(0));
	public static final Pose2d kRobotRightRampExitPose = new Pose2d(new Translation2d(95.25 + kRobotHalfLength, -(97.0 + kRobotHalfWidth - 162.0)), Rotation2d.fromDegrees(0));
	//public static final Pose2d kRobotLeftRampExitPose = new Pose2d(new Translation2d(48.0 + kRobotHalfLength, -75.25 - kRobotHalfWidth), Rotation2d.fromDegrees(0));
	//public static final Pose2d kRobotRightRampExitPose = new Pose2d(new Translation2d(48.0 + kRobotHalfLength, 75.25 + kRobotHalfWidth), Rotation2d.fromDegrees(0));
	
	//Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 21.0;
    public static final double kWheelbaseWidth  = 21.0;
    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);
    
    //Camera Constants
    public static final double kCameraYOffset = 0.25;//0.25
    public static final double kCameraXOffset = kRobotHalfLength - 15.0;
    public static final double kCameraZOffset = 16.45;
    public static final double kCameraYawAngleDegrees = 0.0;//-12.7
    public static final double kCameraPitchAngleDegrees = kIsUsingCompBot ? 14.65 : 14.95;//13.45
    
    //Goal tracker constants
    public static double kMaxGoalTrackAge = 0.5;//0.5
    public static double kMaxTrackerDistance = 60.0;//18.0
    public static double kCameraFrameRate = 90.0;
    public static double kTrackReportComparatorStablityWeight = 1.0;
	public static double kTrackReportComparatorAgeWeight = 1.0;
	public static final double kDefaultCurveDistance = kRobotHalfLength + 36.0;
	public static final double kVisionUpdateDistance = kRobotHalfLength + 75.0;
	public static final double kVisionDistanceStep = 4.0;
	public static final double kClosestVisionDistance = 26.0;//36.0
	public static final double kDefaultVisionTrackingSpeed = 42.0;
	public static final double kCurvedVisionYOffset = 0.375;//1.25

	//Vision Speed Constraint Treemap
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kVisionSpeedTreemap = new InterpolatingTreeMap<>();
	static{
		kVisionSpeedTreemap.put(new InterpolatingDouble(-6.0), new InterpolatingDouble(24.0));
		kVisionSpeedTreemap.put(new InterpolatingDouble(kClosestVisionDistance), new InterpolatingDouble(24.0));
		kVisionSpeedTreemap.put(new InterpolatingDouble(60.0), new InterpolatingDouble(48.0));
		kVisionSpeedTreemap.put(new InterpolatingDouble(300.0), new InterpolatingDouble(48.0));
	}
    
    //Path following constants
    public static final double kPathLookaheadTime = 0.25;  // seconds to look ahead along the path for steering 0.4
	public static final double kPathMinLookaheadDistance = 6.0;  // inches 24.0 (we've been using 3.0)
    
    //Swerve Speed Constants
    public static final double kSwerveDriveMaxSpeed = 28000.0;
    public static final double kSwerveMaxSpeedInchesPerSecond = 12.5 * 12.0;
	public static final double kSwerveRotationMaxSpeed = 1250.0 * 0.8; //The 0.8 is to request a speed that is always achievable
	public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;
    public static final double kSwerveRotationSpeedScalar = ((1.0 / 0.125) - 1.0) / kSwerveMaxSpeedInchesPerSecond;
    
    //Swerve Module Wheel Offsets (Rotation encoder values when the wheels are facing 0 degrees)
	public static final int kFrontRightEncoderStartingPos = kIsUsingCompBot ? -1403 - 1024 : 1740 - 1024;
	public static final int kFrontLeftEncoderStartingPos = kIsUsingCompBot ? -2171 - 1024 : -2895 - 1024;
	public static final int kRearLeftEncoderStartingPos = kIsUsingCompBot ? -1327 - 1024 : -2639 - 1024;
	public static final int kRearRightEncoderStartingPos = kIsUsingCompBot ? -5953 - 1024 : 975 - 1024;
	
	//Swerve Module Positions (relative to the center of the drive base)
	public static final Translation2d kVehicleToModuleZero = new Translation2d(kWheelbaseLength/2, kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleOne = new Translation2d(kWheelbaseLength/2, -kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleTwo = new Translation2d(-kWheelbaseLength/2, -kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleThree = new Translation2d(-kWheelbaseLength/2, kWheelbaseWidth/2);
	
	public static final List<Translation2d> kModulePositions = Arrays.asList(kVehicleToModuleZero,
			kVehicleToModuleOne, kVehicleToModuleTwo, kVehicleToModuleThree);
	
	//Scrub Factors
	public static final boolean kSimulateReversedCarpet = false;
	public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
	public static final double kXScrubFactor = 1.0 / (1.0 - (9549.0 / 293093.0));
	public static final double kYScrubFactor = 1.0 / (1.0 - (4.4736 / 119.9336));

	//Voltage-Velocity equation constants {m, b, x-intercept}
	//First set is the positive direction, second set is negative
	public static final double[][][] kVoltageVelocityEquations = new double[][][]{
		{{1.70, -4.39, 2.58}, {1.83, 5.23, -2.85}}, 
		{{1.59, -3.86, 2.42}, {1.43, 3.09, -2.16}}, 
		{{1.53, -3.66, 2.39}, {1.66, 4.15, -2.50}}, 
		{{1.84, -4.70, 2.56}, {1.85, 5.34, -2.89}}};
	
	//Swerve Odometry Constants
	public static final double kSwerveWheelDiameter = 4.0901; //inches (actual diamter is closer to 3.87, but secondary algorithm prefers 4.0901) 3.76
	public static final double kSwerveDriveEncoderResolution = 4096.0;
	/** The number of rotations the swerve drive encoder undergoes for every rotation of the wheel. */
	public static final double kSwerveEncoderToWheelRatio = 6.0;
	public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
	public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);

	//Ball Intake Constants
	public static final double kIntakeWeakEjectOutput = -0.75;
	public static final double kIntakeEjectOutput = kIsUsingCompBot ? -0.6 : -0.9;
	public static final double kIntakeStrongEjectOutput = -1.0;
	public static final double kIntakingOutput = 1.0;
	public static final double kIntakeWeakHoldingOutput = 2.0/12.0;
	public static final double kIntakeStrongHoldingOutput = 4.0/12.0;
	public static final double kIntakingResuckingOutput = 6.0/12.0;
	public static final double kIntakeRampRate = 0.25;
	public static final double kIntakeClimbOutput = 6.0/12.0;
	public static final double kIntakePullOutput = 12.0/12.0;

	//Ball Carriage Constants
	public static final double kBallCarriageEjectOutput = -0.8;
	public static final double kBallCarriageWeakEjectOutput = -0.5;
	public static final double kBallCarriageReceiveOutput = -0.55;
	public static final double kBallCarriageSuckOutput = 0.35;

	//Disk Intake Constants
	public static final double kPumpHoldOutput = 0.0;
	public static final double kPumpBuildOutput = 1.0;

	//LED Colors
	public static final List<Double> pink = Arrays.asList(255.0, 20.0, 30.0);
	public static final List<Double> blue = Arrays.asList(0.0, 0.0, 255.0);
	public static final List<Double> red = Arrays.asList(255.0, 0.0, 0.0);
	public static final List<Double> orange = Arrays.asList(255.0, 20.0, 0.0);
	public static final List<Double> yellow = Arrays.asList(255.0, 60.0, 0.0);
	public static final List<Double> green = Arrays.asList(0.0, 255.0, 0.0);
	public static final List<Double> purple = Arrays.asList(255.0, 0.0, 255.0);

	//LED Arrays
	public static final List<List<Double>> rainbow = Arrays.asList(red, orange, yellow, green, blue, pink, purple);

	public static final int kLongCANTimeoutMs = 100;

	//Gamepad Ports
	public static final boolean kUseDriveGamepad = false;
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.2;
}
