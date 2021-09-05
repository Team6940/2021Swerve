// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HolonomicDrivetrain;
import edu.wpi.first.wpilibj.GenericHID;

public class HolonomicDriveCommand extends CommandBase {
  /** Creates a new HolonomicDriveCommand. */
  private final HolonomicDrivetrain mDrivetrain;

  public HolonomicDriveCommand(HolonomicDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDrivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  private double deadband(double input) {
		if (Math.abs(input) < 0.05) return 0;
		return input;
	}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = -RobotContainer.m_controller.getY(GenericHID.Hand.kLeft);
		double strafe = RobotContainer.m_controller.getX(GenericHID.Hand.kLeft);
		double rotation = (RobotContainer.m_controller.getX(GenericHID.Hand.kRight));

		forward *= Math.abs(forward);
		strafe *= Math.abs(strafe);
		rotation *= Math.abs(rotation);

		forward = deadband(forward);
		strafe = deadband(strafe);
		rotation = deadband(rotation);

		SmartDashboard.putNumber("Forward", forward);
		SmartDashboard.putNumber("Strafe", strafe);
		SmartDashboard.putNumber("Rotation", rotation);

		mDrivetrain.holonomicDrive(forward, strafe, rotation * 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.stopDriveMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
