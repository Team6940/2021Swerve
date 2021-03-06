// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveDriveModule extends SubsystemBase {
  /** Creates a new SwerveDriveModule. */
  private static final long STALL_TIMEOUT = 2000;

  private long mStallTimeBegin = Long.MAX_VALUE;

  private double mLastError = 0, lastTargetAngle = 0;

  private final int moduleNumber;

  private final double mZeroOffset;

  private final WPI_TalonSRX mAngleMotor;
  private final WPI_TalonFX mDriveMotor;

  Pose2d swerveModulePose = new Pose2d();

  private boolean driveInverted = false;
  private double driveGearRatio = 7;
  private double driveWheelRadius = 2.2;
  private boolean angleMotorJam = false;

  public SwerveDriveModule(int moduleNumber, WPI_TalonSRX angleMotor, WPI_TalonFX driveMotor, double zeroOffset, boolean driveinvert) {
    this.moduleNumber = moduleNumber;

    mAngleMotor = angleMotor;
    mDriveMotor = driveMotor;

    mZeroOffset = zeroOffset;

    driveMotor.setInverted(driveinvert);

    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    angleMotor.setSensorPhase(true);
    angleMotor.config_kP(0, 30, 0);
    angleMotor.config_kI(0, 0.001, 0);
    angleMotor.config_kD(0, 200, 0);
    angleMotor.setNeutralMode(NeutralMode.Brake);
    angleMotor.configClosedLoopPeakOutput(0,0.3,0);
    angleMotor.set(ControlMode.Position, 0);

    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    //driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
    //driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

    driveMotor.config_kP(0, Constants.kGains_Velocit.kP, 0);
    driveMotor.config_kI(0, Constants.kGains_Velocit.kI, 0);
    driveMotor.config_kD(0, Constants.kGains_Velocit.kD, 0);
    driveMotor.config_kF(0, Constants.kGains_Velocit.kF, 0);

    //driveMotor.configMotionCruiseVelocity(640, 0);
    //driveMotor.configMotionAcceleration(200, 0);

    driveMotor.setNeutralMode(NeutralMode.Brake);

    // Set amperage limits
    angleMotor.configContinuousCurrentLimit(30, 0);
    angleMotor.configPeakCurrentLimit(30, 0);
    angleMotor.configPeakCurrentDuration(100, 0);
    angleMotor.enableCurrentLimit(true);

    //driveMotor.configContinuousCurrentLimit(25, 0);
    //driveMotor.configPeakCurrentLimit(25, 0);
    //driveMotor.configPeakCurrentDuration(100, 0);
    //driveMotor.enableCurrentLimit(true);
    
    SmartDashboard.putBoolean("Motor Jammed" + moduleNumber, angleMotorJam);
  }

    private double encoderTicksToInches(double ticks) {
      if (Robot.PRACTICE_BOT) {
          return ticks / 36.65;
      } else {
          return ticks / 35.6;
      }
  }

  private int inchesToEncoderTicks(double inches) {
      if (Robot.PRACTICE_BOT) {
          return (int) Math.round(inches * 36.65);
      } else {
          return (int) Math.round(inches * 35.6);
      }
  }

  public void setPose(Pose2d pose) {
    swerveModulePose = pose;
  }

  /*@Override
  protected void initDefaultCommand() {
      setDefaultCommand(new SwerveModuleCommand(this));
  }*/

  public TalonSRX getAngleMotor() {
      return mAngleMotor;
  }

  /**
   * Get the current angle of the swerve module
   *
   * @return An angle in the range [0, 360)
   */
  public double getCurrentAngle() {
      double angle = mAngleMotor.getSelectedSensorPosition(0) * (360.0 / 4096.0);
      angle -= mZeroOffset;
      angle %= 360;
      if (angle < 0) angle += 360;

      return angle;
  }

  public double getDriveDistance() {
      int ticks = (int)mDriveMotor.getSelectedSensorPosition(0);
      if (driveInverted)
          ticks = -ticks;

      return encoderTicksToInches(ticks);
  }

  public WPI_TalonFX getDriveMotor() {
      return mDriveMotor;
  }

  public double getTargetAngle() {
      return lastTargetAngle;
  }

  public void robotDisabledInit() {
      mStallTimeBegin = Long.MAX_VALUE;
  }

  public void setDriveGearRatio(double ratio) {
      driveGearRatio = ratio;
  }

  public void setDriveInverted(boolean inverted) {
      driveInverted = inverted;
  }

  public double getDriveWheelRadius() {
      return driveWheelRadius;
  }

  public void setDriveWheelRadius(double radius) {
      driveWheelRadius = radius;
  }

  public void setTargetAngle(double targetAngle) {
  //    	if(angleMotorJam) {
  //    		mAngleMotor.set(ControlMode.Disabled, 0);
  //    		return;
  //    	}
    
      lastTargetAngle = targetAngle;

      targetAngle %= 360;

      SmartDashboard.putNumber("Module Target Angle " + moduleNumber, targetAngle % 360);

      targetAngle += mZeroOffset;

      double currentAngle = mAngleMotor.getSelectedSensorPosition(0) * (360.0 / 4096.0);
      double currentAngleMod = currentAngle % 360;
      if (currentAngleMod < 0) currentAngleMod += 360;

      double delta = currentAngleMod - targetAngle;

      if (delta > 180) {
          targetAngle += 360;
      } else if (delta < -180) {
          targetAngle -= 360;
      }

      delta = currentAngleMod - targetAngle;
      if (delta > 90 || delta < -90) {
          if (delta > 90)
              targetAngle += 180;
          else if (delta < -90)
              targetAngle -= 180;
          mDriveMotor.setInverted(false);
      } else {
          mDriveMotor.setInverted(true);
      }

      targetAngle += currentAngle - currentAngleMod;

      double currentError = mAngleMotor.getClosedLoopError(0);
  //        if (Math.abs(currentError - mLastError) < 7.5 &&
  //                Math.abs(currentAngle - targetAngle) > 5) {
  //            if (mStallTimeBegin == Long.MAX_VALUE) {
  //            	mStallTimeBegin = System.currentTimeMillis();
  //            }
  //            if (System.currentTimeMillis() - mStallTimeBegin > STALL_TIMEOUT) {
  //            	angleMotorJam = true;
  //            	mAngleMotor.set(ControlMode.Disabled, 0);
  //            	mDriveMotor.set(ControlMode.Disabled, 0);
  //            	SmartDashboard.putBoolean("Motor Jammed" + moduleNumber, angleMotorJam);
  //            	return;
  //            }
  //        } else {
  //            mStallTimeBegin = Long.MAX_VALUE;
  //        }
      mLastError = currentError;
      targetAngle *= 4096.0 / 360.0;
      mAngleMotor.set(ControlMode.Position, targetAngle);
  }

  public void setTargetDistance(double distance) {
  //    	if(angleMotorJam) {
  //    		mDriveMotor.set(ControlMode.Disabled, 0);
  //    		return;
  //    	}
      if (driveInverted) distance = -distance;

  //        distance /= 2 * Math.PI * driveWheelRadius; // to wheel rotations
  //        distance *= driveGearRatio; // to encoder rotations
  //        distance *= 80; // to encoder ticks

      distance = inchesToEncoderTicks(distance);

      SmartDashboard.putNumber("Module Ticks " + moduleNumber, distance);

      mDriveMotor.set(ControlMode.MotionMagic, distance);
  }

  public void setTargetSpeed(double speed) {
  //    	if(angleMotorJam) {
  //    		mDriveMotor.set(ControlMode.Disabled, 0);
  //    		return;
  //    	}

      double targetVelocity_UnitsPer100ms = speed * 4000.0 * 2048.0 / 600.0;
      //if (driveInverted) speed = -speed;

      if (driveInverted) targetVelocity_UnitsPer100ms = -targetVelocity_UnitsPer100ms;

      //mDriveMotor.set(ControlMode.PercentOutput, speed);

      mDriveMotor.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }

  public void zeroDistance() {
      mDriveMotor.setSelectedSensorPosition(0, 0, 0);
  }

  public void resetMotor() {
    angleMotorJam = false;
    mStallTimeBegin = Long.MAX_VALUE;
    SmartDashboard.putBoolean("Motor Jammed" + moduleNumber, angleMotorJam);
  }

  public void setMotionConstraints(double maxAcceleration, double maxVelocity) {
      mDriveMotor.configMotionAcceleration(inchesToEncoderTicks(maxAcceleration * 12) / 10, 0);
      mDriveMotor.configMotionCruiseVelocity(inchesToEncoderTicks(maxVelocity * 12) / 10, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
