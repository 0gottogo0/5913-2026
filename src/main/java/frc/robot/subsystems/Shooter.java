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

public class Shooter extends SubsystemBase {

  	private TalonFX shooter1 = new TalonFX(1); // create constants and change motor names
  	private TalonFX shooter2 = new TalonFX(2);
  	private TalonFX shooter3 = new TalonFX(3);
  	private TalonFX shooter4 = new TalonFX(4);
  	private TalonFX shooter5 = new TalonFX(5);
  	private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

	private PIDController shooterSpeedPID = new PIDController(0.5, 0, 0); // tune

  	private boolean shouldShoot = false; // replace with enum?
  	private double targetSpeed = 0;
	private double shooterSpeedPIDOutput = 0;

  	public Shooter() {
  	  	shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		
  	  	shooter1.clearStickyFaults();
  	  	shooter1.getConfigurator().apply(shooterConfig);
		
  	  	shooter2.clearStickyFaults();
  	  	shooter2.getConfigurator().apply(shooterConfig);
		
  	  	shooter3.clearStickyFaults();
  	  	shooter3.getConfigurator().apply(shooterConfig);
		
  	  	shooter4.clearStickyFaults();
  	  	shooter4.getConfigurator().apply(shooterConfig);
		
  	  	shooter5.clearStickyFaults();
  	  	shooter5.getConfigurator().apply(shooterConfig);
		
  	  	//shooter2.setControl(new Follower(shooter1.getDeviceID(), false)); // <-- do this but with no errors
  	}

  	@Override
  	public void periodic() {
  	  	if (shouldShoot) {
			shooterSpeedPIDOutput = shooterSpeedPID.calculate(getShooterSpeed(), targetSpeed);
			shooter1.set(shooterSpeedPIDOutput);
		} else {
			shooter1.set(targetSpeed);
		}

		SmartDashboard.putNumber("Shooter RPS", getShooterSpeed());
  	}

	private double getShooterSpeed() {
		return shooter1.getRotorVelocity().getValueAsDouble();
	}

  	public void startShooting(double speed) {
  		targetSpeed = speed;
  	  	shouldShoot = true;
  	}

  	public void stopShooting() {
		targetSpeed = 0;
  	  	shouldShoot = false;
  	}

	public void shootOpenLoop(double speed) {
		targetSpeed = speed;
		shouldShoot = false; // again should we chang this to an enum, naming is misleading
	}
}
