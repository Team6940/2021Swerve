// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final double width;
	private final double length;
  private double speedMultiplier = 0.65;
  
  public Drivetrain(double width, double length) {
    this.width = width;
		this.length = length;
  }

  
	public final double getWidth() {
		return width;
	}

	public final double getLength() {
		return length;
	}

	public double getSpeedMultiplier() {
		return speedMultiplier;
	}

	public void setSpeedMultiplier(double speedMultiplier) {
		this.speedMultiplier = speedMultiplier;
	}

	public abstract double getMaxAcceleration();

	public abstract double getMaxVelocity();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
