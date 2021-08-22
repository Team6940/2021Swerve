// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int ELEVATOR_SHIFTER = 3;
    public static final int ELEVATOR_LOCKER = 2;

	public static final int GATHERER_LEFT_SOLENOID = 1;
	public static final int GATHERER_RIGHT_SOLENOID = 0;

	public static final int GATHERER_LEFT_MOTOR = 30;
    public static final int GATHERER_RIGHT_MOTOR = 29;

	public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 1;
	public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 2;
	public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 4;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 3;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 5;
    public static final int DRIVETRAIn_BACK_LEFT_ANGLE_MOTOR = 6;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 8;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 7;


	public static final int ELEVATOR_MASTER_MOTOR = 28;
	public static final int ELEVATOR_SLAVE_MOTOR = 27;

	public static final int CARRIAGE_LEFT_MOTOR = 21;
	public static final int CARRIAGE_RIGHT_MOTOR = 22;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
     * 
	 * 	                                    			  kP   	 kI    kD      kF          Iz    PeakOut */
	public final static Gains kGains_Velocit  = new Gains( 0.1, 0.001, 5, 1023.0/20660.0,  300,  1.00);

	public static final double kTrackWidth = 13.5; //This part need to be adjusted
	// Distance between centers of right and left wheels on robot
	public static final double kWheelBase = 13.5;  //This part need to be adjusted
	// Distance between front and back wheels on robot
	public static final SwerveDriveKinematics kDriveKinematics =
		new SwerveDriveKinematics(
			new Translation2d(kWheelBase / 2, kTrackWidth / 2),
			new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
			new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
			new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

	public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
}
