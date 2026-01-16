// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.ShooterConstants.State;

public class Shooter extends SubsystemBase {

  	private TalonFX shooter = new TalonFX(30); // create constants and change motor names
  	private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

	private PIDController shooterSpeedPID = new PIDController(0.01, 0, 0); // tune

  	private double targetSpeed = 0;
	private double shooterSpeedPIDOutput = 0;

	public State state = ShooterConstants.State.Idle;

  	public Shooter() {
  	  	shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		
  	  	shooter.clearStickyFaults();
  	  	shooter.getConfigurator().apply(shooterConfig);
  	}

  	@Override
  	public void periodic() {
  	  	if (state == State.Shoot) {
			shooterSpeedPIDOutput = shooterSpeedPID.calculate(getShooterSpeed(), targetSpeed);
			shooter.set(shooterSpeedPIDOutput);
		} else if (state == State.Idle){
			shooterSpeedPIDOutput = shooterSpeedPID.calculate(getShooterSpeed(), ShooterConstants.IdleRPS);
			shooter.set(shooterSpeedPIDOutput);
		} else if (state == State.Unstick){
			shooterSpeedPIDOutput = shooterSpeedPID.calculate(getShooterSpeed(), ShooterConstants.UnstickRPS);
			shooter.set(shooterSpeedPIDOutput);
		} else {
			shooter.set(targetSpeed);
		}

		SmartDashboard.putNumber("Shooter RPS", getShooterSpeed());
  	}

	private double getShooterSpeed() {
		return shooter.getRotorVelocity().getValueAsDouble();
	}

  	public void startShooting(double speedInRPS) {
  		targetSpeed = speedInRPS;
  	  	state = State.Shoot;
  	}

  	public void stopShooting() {
		targetSpeed = ShooterConstants.IdleRPS;
  	  	state = State.Idle;
  	}

	public void setShooterState(State stateToChangeTo, double speedInRPS) {
		state = stateToChangeTo;
		targetSpeed = speedInRPS;
	}

	public void setShooterDumbControl(double speedInPercent) {
		targetSpeed = speedInPercent;
		state = State.DumbControl;
	}
}
