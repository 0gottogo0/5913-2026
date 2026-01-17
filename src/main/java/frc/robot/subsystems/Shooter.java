// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.ShooterConstants.State;

public class Shooter extends SubsystemBase {

  	private TalonFX shooter = new TalonFX(ShooterConstants.MotorID);
  	private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

	private VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0);

  	private double targetSpeed = 0;
	private double shooterSpeedPIDOutput = 0;

	public State state = ShooterConstants.State.Idle;

  	public Shooter() {
  	  	shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		shooterConfig.Slot0.kV = ShooterConstants.PIDkV;
		shooterConfig.Slot0.kP = ShooterConstants.PIDkP;
		shooterConfig.Slot0.kI = ShooterConstants.PIDkI;
		shooterConfig.Slot0.kD = ShooterConstants.PIDkD;

  	  	shooter.clearStickyFaults();
  	  	shooter.getConfigurator().apply(shooterConfig);
	}

  	@Override
  	public void periodic() {
  	  	if (state == State.Idle) {
			shooter.set(0.00);
		} else if (state == State.Spinup) {
			// Get to target RPS
			shooter.setControl(shooterVelocityVoltage.withVelocity(targetSpeed));
		} else if (state == State.Shoot) {
			// Overshoot target RPS due to heavy ball compression slowing down shooter
			shooter.setControl(shooterVelocityVoltage.withVelocity(targetSpeed + ShooterConstants.ShootRPSAdjustment));
		} else if (state == State.OverTorque){
			// Really overshoot target RPS due to many balls doing many slowing
			shooter.setControl(shooterVelocityVoltage.withVelocity(targetSpeed + ShooterConstants.OverTorqueRPSAdjustment));
		} else if (state == State.Unstick){
			// Shoot shooter incase of stuck ball
			shooter.setControl(shooterVelocityVoltage.withVelocity(ShooterConstants.UnstickRPS));
		} else {
			shooter.set(targetSpeed);
		}

		SmartDashboard.putNumber("Shooter RPS", getShooterSpeed());
		SmartDashboard.putNumber("Shooter Target RPS", targetSpeed);
		SmartDashboard.putNumber("Shooter Target Diff", targetSpeed - getShooterSpeed());
		SmartDashboard.putNumber("Shooter Target Percentage", Math.abs(getShooterSpeed() / targetSpeed - 1));
		SmartDashboard.putNumber("Shooter PID Output", shooterSpeedPIDOutput);

		SmartDashboard.putString("Shooter State", state.toString());

		SmartDashboard.putBoolean("Shooter At Speed", isShooterAtSpeed());
  	}

	private double getShooterSpeed() {
		return shooter.getRotorVelocity().getValueAsDouble();
	}

	public void setShooterState(State stateToChangeTo, double speedInRPS) {
		state = stateToChangeTo;
		targetSpeed = speedInRPS;
	}

	public void setShooterDumbControl(double speedInPercent) {
		targetSpeed = speedInPercent;
		state = State.DumbControl;
	}

	public boolean isShooterAtSpeed() {
		if (state == State.Spinup || state == State.Shoot) {
			return Math.abs(getShooterSpeed() / targetSpeed - 1) < ShooterConstants.RPSThresholdPercent;
		} else {
			return false;
		}
	}
}
