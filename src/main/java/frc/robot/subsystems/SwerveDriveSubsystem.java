// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveDriveSubsystem extends HolonomicDrivetrain {
  /** Creates a new SwerveDriveSubsystem. */
  //Unit is inch
  public static final double WHEELBASE = 13.5;  // Swerve bot: 14.5 Comp bot: 20.5
  public static final double TRACKWIDTH = 13.5; // Swerve bot: 13.5 Comp bot: 25.5

  public static final double WIDTH = 20;  // Swerve bot: 20 Comp bot: 37
  public static final double LENGTH = 19; // Swerve bot: 19 Comp bot: 32

  /*
  * 0 is Front Right
  * 1 is Front Left
  * 2 is Back Left
  * 3 is Back Right
  */
  private SwerveDriveModule[] mSwerveModules;

  private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);

  private final SwerveDriveOdometry m_odometry =
  new SwerveDriveOdometry(Constants.kDriveKinematics, mNavX.getRotation2d());


  public SwerveDriveSubsystem() {
    super(WIDTH, LENGTH);
    zeroGyro();

    if (Robot.PRACTICE_BOT) {
        mSwerveModules = new SwerveDriveModule[] {
                new SwerveDriveModule(0, new WPI_TalonSRX(3), new WPI_TalonFX(4), 255.5859),
                new SwerveDriveModule(1, new WPI_TalonSRX(6), new WPI_TalonFX(5), 338.906),
                new SwerveDriveModule(2, new WPI_TalonSRX(2), new WPI_TalonFX(1), 13.359),
                new SwerveDriveModule(3, new WPI_TalonSRX(7), new WPI_TalonFX(8), 15.82),
        };

        mSwerveModules[0].setDriveInverted(true);
        mSwerveModules[3].setDriveInverted(true);
    } else {
        mSwerveModules = new SwerveDriveModule[] {
                new SwerveDriveModule(0,
                        new WPI_TalonSRX(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR),
                        new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR),
                        0),
                new SwerveDriveModule(1,
                        new WPI_TalonSRX(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR),
                        new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR),
                        164.09),
                new SwerveDriveModule(2,
                        new WPI_TalonSRX(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR),
                        new WPI_TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR),
                        0),
               
                new SwerveDriveModule(3,
                        new WPI_TalonSRX(Constants.DRIVETRAIn_BACK_LEFT_ANGLE_MOTOR),
                        new WPI_TalonFX(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR),
                        -167.70),
        };

        mSwerveModules[0].setDriveInverted(true);
        mSwerveModules[3].setDriveInverted(true);
    }

    for (SwerveDriveModule module : mSwerveModules) {
        module.setTargetAngle(0);
        module.setDriveGearRatio(7);
        module.setDriveWheelRadius(module.getDriveWheelRadius() * 1.05);
        module.setMotionConstraints(getMaxAcceleration(), getMaxVelocity());
    }
  }

  public AHRS getNavX() {
    return mNavX;
}

public double getGyroAngle() {
    double angle = mNavX.getAngle() - getAdjustmentAngle();
    angle %= 360;
    if (angle < 0) angle += 360;

    if (Robot.PRACTICE_BOT) {
        return angle;
    } else {
        return 360 - angle;
    }
}

public double getGyroRate() {
    return mNavX.getRate();
}

public double getRawGyroAngle() {
    double angle = mNavX.getAngle();
    angle %= 360;
    if (angle < 0) angle += 360;

    return angle;
}

public SwerveDriveModule getSwerveModule(int i) {
    return mSwerveModules[i];
}

public void holonomicDrive(double forward, double strafe, double rotation, boolean fieldOriented) {
    forward *= getSpeedMultiplier();
    strafe *= getSpeedMultiplier();

    strafe *= -1;
    //rotation *= -1;
    

    if (fieldOriented) {
        double angleRad = Math.toRadians(getGyroAngle());
        double temp = forward * Math.cos(angleRad) +
                strafe * Math.sin(angleRad);
        strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
        forward = temp;
    }

    double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
    double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
    double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
    double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

    double[] angles = new double[]{
            Math.atan2(b, c) * 180 / Math.PI,
            Math.atan2(b, d) * 180 / Math.PI,
            Math.atan2(a, d) * 180 / Math.PI,
            Math.atan2(a, c) * 180 / Math.PI
    };

    double[] speeds = new double[]{
            Math.sqrt(b * b + c * c),
            Math.sqrt(b * b + d * d),
            Math.sqrt(a * a + d * d),
            Math.sqrt(a * a + c * c)
    };

    double max = speeds[0];

    for (double speed : speeds) {
        if (speed > max) {
            max = speed;
        }
    }

    if(max > 1){
        for(int i = 0; i < 4; i++){
            speeds[i] /= max;
        }
    }

    for (int i = 0; i < 4; i++) {
        if (Math.abs(forward) > 0.05 ||
                Math.abs(strafe) > 0.05 ||
                Math.abs(rotation) > 0.05) {
            mSwerveModules[i].setTargetAngle(angles[i] + 180);
        } else {
            mSwerveModules[i].setTargetAngle(mSwerveModules[i].getTargetAngle());
        }
        mSwerveModules[i].setTargetSpeed(speeds[i]);
    }
}

    public void stopDriveMotors() {
        for (SwerveDriveModule module : mSwerveModules) {
            module.setTargetSpeed(0);
        }
    }

    public void resetMotors() {
    for(int i = 0; i < mSwerveModules.length; i++) {
        mSwerveModules[i].resetMotor();
    }
    }

    public SwerveDriveModule[] getSwerveModules() {
        return mSwerveModules;
    }

    public double getMaxAcceleration() {
        return 5.5;
    }

    public double getMaxVelocity() {
        return 10;
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
        }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, mNavX.getRotation2d());
      }

        /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveModuleState[] state = desiredStates;
        for(int i  = 0; i < 4; i++){
            mSwerveModules[i].setTargetAngle(state[i].angle.getDegrees() + 180);
            mSwerveModules[i].setTargetSpeed(state[i].speedMetersPerSecond * 0.1);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
