// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterV2 extends SubsystemBase {

	// Placeholder names

	// Kraken X60
	private TalonFX right3 = new TalonFX(50);
    private TalonFXConfiguration right3Config = new TalonFXConfiguration();

	// Kraken X60
	private TalonFX left4 = new TalonFX(51);
    private TalonFXConfiguration left4Config = new TalonFXConfiguration();

	// Kraken X60
  	private TalonFX right4 = new TalonFX(52);
  	private TalonFXConfiguration right4Config = new TalonFXConfiguration();

	// Kraken X44
	private TalonFX left2 = new TalonFX(53);
  	private TalonFXConfiguration left2Config = new TalonFXConfiguration();

	private VelocityVoltage right3VelocityVoltage = new VelocityVoltage(0);
    private VelocityVoltage left4VelocityVoltage = new VelocityVoltage(0);
	private VelocityVoltage right4VelocityVoltage = new VelocityVoltage(0);
    private VelocityVoltage left2VelocityVoltage = new VelocityVoltage(0);

	private double right3TargetSpeed = 0.00;
	private double left4TargetSpeed = 0.00;
  	private double right4TargetSpeed = 0.00;
	private double left2TargetSpeed = 0.00;

	public State state = State.Idle;

  	public ShooterV2() {
	    right3Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
	    right3Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        right3Config.Slot0.kV = 0.00;
		right3Config.Slot0.kP = 0.00;
		right3Config.Slot0.kI = 0.00;
		right3Config.Slot0.kD = 0.00;

		right3.clearStickyFaults();
		right3.getConfigurator().apply(right3Config);

        left4Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	left4Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        left4Config.Slot0.kV = 0.00;
		left4Config.Slot0.kP = 0.00;
		left4Config.Slot0.kI = 0.00;
		left4Config.Slot0.kD = 0.00;

        left4.clearStickyFaults();
  	  	left4.getConfigurator().apply(left4Config);

  	  	right4Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	right4Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        right4Config.Slot0.kV = 0.00;
		right4Config.Slot0.kP = 0.00;
		right4Config.Slot0.kI = 0.00;
		right4Config.Slot0.kD = 0.00;

  	  	right4.clearStickyFaults();
  	  	right4.getConfigurator().apply(right4Config);

        left2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	left2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        left2Config.Slot0.kV = 0.00;
		left2Config.Slot0.kP = 0.00;
		left2Config.Slot0.kI = 0.00;
		left2Config.Slot0.kD = 0.00;

  	  	left2.clearStickyFaults();
  	  	left2.getConfigurator().apply(left2Config);
	}

  	@Override
  	public void periodic() {
  	  	switch (state) {
			case Idle:
				right3.set(0.00);
				left4.set(0.00);
				right4.set(0.00);
				left2.set(0.00);
				break;
			case Spinup:
				right3.setControl(right3VelocityVoltage.withVelocity(right3TargetSpeed));
				left4.setControl(left4VelocityVoltage.withVelocity(left4TargetSpeed));
				right4.setControl(right4VelocityVoltage.withVelocity(right4TargetSpeed));
				left2.setControl(left2VelocityVoltage.withVelocity(left2TargetSpeed));
				break;
			case Shoot:
				right3.setControl(right3VelocityVoltage.withVelocity(right3TargetSpeed));
				left4.setControl(left4VelocityVoltage.withVelocity(left4TargetSpeed));
				right4.setControl(right4VelocityVoltage.withVelocity(right4TargetSpeed));
				left2.setControl(left2VelocityVoltage.withVelocity(left2TargetSpeed));
				break;
			case Unstick: // Get rid of this
				right3.set(0.00);
				left4.set(0.00);
				right4.set(0.00);
				left2.set(0.00);
				break;
			case DumbControl:
				right3.set(-right3TargetSpeed);
				left4.set(-left4TargetSpeed);
				right4.set(right4TargetSpeed);
				left2.set(left2TargetSpeed);
				break;
		} 

		SmartDashboard.putString("ShooterV2 State", state.toString());
  	}

	/**
	 * Sets the state of the shooter.
	 * <p> 
	 * If wanting to control the shooter without PID
	 * then use setShooterDumbControl()
	 * 
	 * @param stateToChangeTo Using ShooterConstants.State
	 *
	 * @param bottomSpeedinRPS The target speed to set in rps.
	 * 					 		   When setting the state to idle,
	 * 					 		   target speed is not used and can
	 * 					 		   be set to 0.
	 * 
	 * @param topSpeedInRPS The target speed to set in rps.
	 * 					 		When setting the state to idle,
	 * 					 		target speed is not used and can
	 * 					 		be set to 0.	
	 * 
	 * @param hoodSpeedInRPS The target speed to set in rps.
	 * 					     When setting the state to idle,
	 * 					     target speed is not used and can
	 * 					     be set to 0.
	 */
	public void setShooterState(State stateToChangeTo, double bottomSpeedinRPS, double topSpeedInRPS, double hoodSpeedInRPS) {
        right3TargetSpeed = bottomSpeedinRPS;
		left4TargetSpeed = topSpeedInRPS;
        right4TargetSpeed = topSpeedInRPS;
        left2TargetSpeed = hoodSpeedInRPS;
		state = stateToChangeTo;
	}

	/**
	 * Sets the state of the shooter to DumbControl
	 * <p>
	 * Used if want to control the shooter open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param bottomSpeedInPercent The speed to control in percent
	 * 
	 * @param topSpeedInPercent The speed to control in percent
	 * 
	 * @param hoodSpeedInPercent The speed to control in percent
	 */
	public void setShooterDumbControl(double bottomSpeedInPercent, double topSpeedInPercent, double hoodSpeedInPercent) {
		right3TargetSpeed = bottomSpeedInPercent;
		left4TargetSpeed = topSpeedInPercent;
        right4TargetSpeed = topSpeedInPercent;
		left2TargetSpeed = hoodSpeedInPercent;
		state = State.DumbControl;
	}
}
