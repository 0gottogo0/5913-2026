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
import frc.robot.constants.Constants.ShooterConstants.State;

public class Shooter extends SubsystemBase {

    // Kraken X44
    private TalonFX bottomRollers = new TalonFX(BottomRollersID);
    private TalonFXConfiguration bottomRollersConfig = new TalonFXConfiguration();

	// Kraken X60
	// 3"
	private TalonFX feeder = new TalonFX(FeederMotorID);
	private TalonFXConfiguration feederConfig = new TalonFXConfiguration();

	// Kraken X60
	// Bottom 4"
	private TalonFX bottomShooter = new TalonFX(BottomMotorID);
    private TalonFXConfiguration bottomShooterConfig = new TalonFXConfiguration();

	// Reminder: Front is the intake. Why? Idrfk.

	// Kraken X60
	// Top 4"
  	private TalonFX leftShooter = new TalonFX(LeftMotorID);
  	private TalonFXConfiguration leftShooterConfig = new TalonFXConfiguration();

	// Kraken X60
	// Top 4"
  	private TalonFX rightShooter = new TalonFX(RightMotorID);
  	private TalonFXConfiguration rightShooterConfig = new TalonFXConfiguration();

	// Kraken X44
	// 2"
	private TalonFX hoodShooter = new TalonFX(HoodMotorID);
  	private TalonFXConfiguration hoodShooterConfig = new TalonFXConfiguration();

	// Guess on why we use a pid on the bottom
	// rollers? I'll wait...
	private VelocityVoltage rollersVelocityVoltage = new VelocityVoltage(0);
	private VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0);
    private VelocityVoltage bottomShooterVelocityVoltage = new VelocityVoltage(0);
	private VelocityVoltage leftShooterVelocityVoltage = new VelocityVoltage(0);
	private VelocityVoltage rightShooterVelocityVoltage = new VelocityVoltage(0);
    private VelocityVoltage hoodShooterVelocityVoltage = new VelocityVoltage(0);

	private double rollersTargetSpeed = 0.00;
	private double feederTargetSpeed = 0.00;
	private double bottomTargetSpeed = 0.00;
  	private double topTargetSpeed = 0.00;
	private double hoodTargetSpeed = 0.00;

	public State state = State.Idle;

  	public Shooter() {
		bottomRollersConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	bottomRollersConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		bottomRollersConfig.Slot0.kV = BeltsPIDkV;
		bottomRollersConfig.Slot0.kP = BeltsPIDkP;
		bottomRollersConfig.Slot0.kI = BeltsPIDkI;
		bottomRollersConfig.Slot0.kD = BeltsPIDkD;

  	  	bottomRollers.clearStickyFaults();
  	  	bottomRollers.getConfigurator().apply(bottomRollersConfig);

		feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		feederConfig.Slot0.kV = FeederPIDkV;
		feederConfig.Slot0.kP = FeederPIDkP;
		feederConfig.Slot0.kI = FeederPIDkI;
		feederConfig.Slot0.kD = FeederPIDkD;

        feeder.clearStickyFaults();
  	  	feeder.getConfigurator().apply(feederConfig);

        bottomShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	bottomShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		bottomShooterConfig.Slot0.kV = BottomShooterPIDkV;
		bottomShooterConfig.Slot0.kP = BottomShooterPIDkP;
		bottomShooterConfig.Slot0.kI = BottomShooterPIDkI;
		bottomShooterConfig.Slot0.kD = BottomShooterPIDkD;

        bottomShooter.clearStickyFaults();
  	  	bottomShooter.getConfigurator().apply(bottomShooterConfig);

  	  	leftShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	leftShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		leftShooterConfig.Slot0.kV = LeftShooterPIDkV;
		leftShooterConfig.Slot0.kP = LeftShooterPIDkP;
		leftShooterConfig.Slot0.kI = LeftShooterPIDkI;
		leftShooterConfig.Slot0.kD = LeftShooterPIDkD;

  	  	leftShooter.clearStickyFaults();
  	  	leftShooter.getConfigurator().apply(leftShooterConfig);

		rightShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	rightShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		rightShooterConfig.Slot0.kV = RightShooterPIDkV;
		rightShooterConfig.Slot0.kP = RightShooterPIDkP;
		rightShooterConfig.Slot0.kI = RightShooterPIDkI;
		rightShooterConfig.Slot0.kD = RightShooterPIDkD;

  	  	rightShooter.clearStickyFaults();
  	  	rightShooter.getConfigurator().apply(rightShooterConfig);

        hoodShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	hoodShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		hoodShooterConfig.CurrentLimits.SupplyCurrentLimit = 80;
		hoodShooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		hoodShooterConfig.Slot0.kV = HoodShooterPIDkV;
		hoodShooterConfig.Slot0.kP = HoodShooterPIDkP;
		hoodShooterConfig.Slot0.kI = HoodShooterPIDkI;
		hoodShooterConfig.Slot0.kD = HoodShooterPIDkD;

  	  	hoodShooter.clearStickyFaults();
		hoodShooter.getConfigurator().apply(hoodShooterConfig);
	}

  	@Override
  	public void periodic() {
  	  	switch (state) {
			case Idle:
                bottomRollers.set(0.00);
				feeder.set(0.00);
				bottomShooter.set(0.00);
				leftShooter.set(0.00);
				rightShooter.set(0.00);
				hoodShooter.set(0.00);
				break;
			case Spinup:
                bottomRollers.set(0.00);
				feeder.set(0.00);
				bottomShooter.setControl(bottomShooterVelocityVoltage.withVelocity(topTargetSpeed));
				leftShooter.setControl(leftShooterVelocityVoltage.withVelocity(topTargetSpeed));
				rightShooter.setControl(rightShooterVelocityVoltage.withVelocity(-topTargetSpeed));
				hoodShooter.setControl(hoodShooterVelocityVoltage.withVelocity(-hoodTargetSpeed));
				break;
			case Shoot:
                bottomRollers.setControl(rollersVelocityVoltage.withVelocity(BeltsSpeed));
				feeder.setControl(feederVelocityVoltage.withVelocity(FeedSpeed));
				bottomShooter.setControl(bottomShooterVelocityVoltage.withVelocity(topTargetSpeed));
				leftShooter.setControl(leftShooterVelocityVoltage.withVelocity(topTargetSpeed));
				rightShooter.setControl(rightShooterVelocityVoltage.withVelocity(-topTargetSpeed));
				hoodShooter.setControl(hoodShooterVelocityVoltage.withVelocity(-hoodTargetSpeed));
				break;
			case DumbControl:
                bottomRollers.set(rollersTargetSpeed);
				feeder.set(feederTargetSpeed);
				bottomShooter.set(bottomTargetSpeed);
				leftShooter.set(topTargetSpeed);
				rightShooter.set(-topTargetSpeed);
				hoodShooter.set(-hoodTargetSpeed);
				break;
		}

        SmartDashboard.putNumber("Belts RPS", getBeltsSpeed());
		SmartDashboard.putNumber("Belts Target RPS", rollersTargetSpeed);
		SmartDashboard.putNumber("Belts Target Diff", rollersTargetSpeed - getBeltsSpeed());

        SmartDashboard.putNumber("Feeder Shooter RPS", getFeederShooterSpeed());
		SmartDashboard.putNumber("Feeder Shooter Target RPS", feederTargetSpeed);
		SmartDashboard.putNumber("Feeder Shooter Target Diff", feederTargetSpeed - getFeederShooterSpeed());

        SmartDashboard.putNumber("Bottom Shooter RPS", getBottomShooterSpeed());
		SmartDashboard.putNumber("Bottom Shooter Target RPS", bottomTargetSpeed);
		SmartDashboard.putNumber("Bottom Shooter Target Diff", (bottomTargetSpeed) - getBottomShooterSpeed());

		SmartDashboard.putNumber("Left Shooter RPS", getLeftShooterSpeed());
		SmartDashboard.putNumber("Left Shooter Target RPS", topTargetSpeed);
		SmartDashboard.putNumber("Left Shooter Target Diff", topTargetSpeed - getLeftShooterSpeed());
        
        SmartDashboard.putNumber("Right Shooter RPS", getRightShooterSpeed());
		SmartDashboard.putNumber("Right Shooter Target RPS", topTargetSpeed);
		SmartDashboard.putNumber("Right Shooter Target Diff", topTargetSpeed - getRightShooterSpeed());

		SmartDashboard.putNumber("Hood Shooter RPS", getHoodShooterSpeed());
		SmartDashboard.putNumber("Hood Shooter Target RPS", hoodTargetSpeed);
		SmartDashboard.putNumber("Hood Shooter Target Diff", hoodTargetSpeed - getHoodShooterSpeed());
        
		SmartDashboard.putString("Shooter State", state.toString());

		SmartDashboard.putBoolean("Shooter At Speed", isShooterAtSpeed());
  	}
    
	/**
	 * Returns the speed of the belts motor
	 * 
	 * @return Speed in RPS
	 */
    private double getBeltsSpeed() {
        return bottomRollers.getRotorVelocity().getValueAsDouble();
    }

	/**
     * Returns the speed of the feeder shooter motor
     * 
     * @return Speed in RPS
     */
    private double getFeederShooterSpeed() {
        return feeder.getRotorVelocity().getValueAsDouble();
	}

    /**
     * Returns the speed of the bottom shooter motor
     * 
     * @return Speed in RPS
     */
    private double getBottomShooterSpeed() {
        return bottomShooter.getRotorVelocity().getValueAsDouble();
    }

	/**
	 * Returns the speed of the left shooter motor
	 * 
	 * @return Speed in RPS
	 */
	private double getLeftShooterSpeed() {
		return leftShooter.getRotorVelocity().getValueAsDouble();
	}

    /**
	 * Returns the speed of the right shooter motor
	 * 
	 * @return Speed in RPS
	 */
	private double getRightShooterSpeed() {
		return rightShooter.getRotorVelocity().getValueAsDouble();
	}

    /**
	 * Returns the speed of the top shooter motor
	 * 
	 * @return Speed in RPS
	 */
	private double getHoodShooterSpeed() {
		return hoodShooter.getRotorVelocity().getValueAsDouble();
	}

	/**
	 * Sets the state of the shooter.
	 * <p> 
	 * If wanting to control the shooter without PID
	 * then use setShooterDumbControl()
	 * 
	 * @param stateToChangeTo Using ShooterConstants.State

	 * @param speedInRPS The target speed to set in rps.
	 * 					 When setting the state to idle,
	 * 					 target speed is not used and can
	 * 					 be set to 0.	
	 * 
	 * @param hoodSpeedInRPS The target speed to set in rps.
	 * 					     When setting the state to idle,
	 * 					     target speed is not used and can
	 * 					     be set to 0.
	 */
	public void setShooterState(State stateToChangeTo, double speedInRPS, double hoodSpeedInRPS) {
        topTargetSpeed = speedInRPS;
		hoodTargetSpeed = hoodSpeedInRPS;
		state = stateToChangeTo;
	}

	/**
	 * Sets the state of the shooter to DumbControl
	 * <p>
	 * Used if want to control the shooter open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param feedSpeedInPercent The speed to control in percent
	 * 
	 * @param topSpeedInPercent The speed to control in percent
	 * 
	 * @param hoodSpeedInPercent The speed to control in percent
	 */
	public void setShooterDumbControl(double feedSpeedInPercent, double topSpeedInPercent, double hoodSpeedInPercent) {
		rollersTargetSpeed = feedSpeedInPercent;
        feederTargetSpeed = feedSpeedInPercent;
		bottomTargetSpeed = topSpeedInPercent;
        topTargetSpeed = topSpeedInPercent;
		hoodTargetSpeed = hoodSpeedInPercent;
		state = State.DumbControl;
	}

	/**
	 * Sets the state of the shooter to DumbControl
	 * <p>
	 * Used if want to control the shooter open loop without
	 * the PID. Uses the TalonFX .set() function 
     * 
	 * @param beltsSpeedInPercent The speed to control in percent
     * 
	 * @param feedSpeedInPercent The speed to control in percent
	 * 
	 * @param bottomSpeedInPercent The speed to control in percent
	 * 
	 * @param topSpeedInPercent The speed to control in percent
	 * 
	 * @param hoodSpeedInPercent The speed to control in percent
	 */
	public void setShooterDumbControl(double beltsSpeedInPercent, double feedSpeedInPercent, double bottomSpeedInPercent, double topSpeedInPercent, double hoodSpeedInPercent) {
        rollersTargetSpeed = beltsSpeedInPercent;
        feederTargetSpeed = feedSpeedInPercent;
        bottomTargetSpeed = bottomSpeedInPercent;
		topTargetSpeed = topSpeedInPercent;
		hoodTargetSpeed = hoodSpeedInPercent;
		state = State.DumbControl;
	}

	/**
	 * Checks if the shooter is up to speed then
	 * returns a boolean, uses ShooterConstants.RPSThreshold
	 * 
	 * @return True if the shooter is up to speed
	 */
	public boolean isShooterAtSpeed() {
		if (state == State.Spinup || state == State.Shoot) {
            // Idk just get the left one
			return getLeftShooterSpeed() > (topTargetSpeed - RPSThreshold);
		} else {
			return false;
		}
	}
}
