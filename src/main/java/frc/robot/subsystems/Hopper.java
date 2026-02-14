// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.HopperConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.Constants.HopperConstants.State;

public class Hopper extends SubsystemBase {

    // Kraken X44
    private TalonFX belts = new TalonFX(BeltsID);
    private TalonFXConfiguration beltsConfig = new TalonFXConfiguration();

    // Kraken X44
    private TalonFX hopper = new TalonFX(HopperID);
    private TalonFXConfiguration hopperConfig = new TalonFXConfiguration();

    private VelocityVoltage beltsVelocityVoltage = new VelocityVoltage(0);
    private PositionVoltage hopperPositionVoltage = new PositionVoltage(0);

    private double targetBeltsSpeed = 0.00;
    private double targetHopperSpeed = 0.00;
    private double hopperLLTA = LimelightHelpers.getTA(LimelightHopper);

    public State state = State.Idle;

    public Hopper() {
        beltsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	beltsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		beltsConfig.Slot0.kV = BeltsPIDkV;
		beltsConfig.Slot0.kP = BeltsPIDkP;
		beltsConfig.Slot0.kI = BeltsPIDkI;
		beltsConfig.Slot0.kD = BeltsPIDkD;

  	  	belts.clearStickyFaults();
  	  	belts.getConfigurator().apply(beltsConfig);

        hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  	  	hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		hopperConfig.Slot0.kV = HopperPIDkV;
		hopperConfig.Slot0.kP = HopperPIDkP;
		hopperConfig.Slot0.kI = HopperPIDkI;
		hopperConfig.Slot0.kD = HopperPIDkD;

    	hopper.clearStickyFaults();
  	  	hopper.getConfigurator().apply(hopperConfig);
    }

    @Override
    public void periodic() {
         if (state == State.Idle) {
            belts.set(0.00);
            hopper.set(0.00);
        } else if (state == State.Intake) {
            belts.setControl(beltsVelocityVoltage.withVelocity(IntakeingSpeed));
            hopper.setControl(hopperPositionVoltage.withPosition(null));
        } else if (state == State.Outtake) {
            belts.setControl(beltsVelocityVoltage.withVelocity(-IntakeingSpeed));
            hopper.setControl(hopperPositionVoltage.withPosition(null));
        } else {
            belts.set(targetBeltsSpeed);
            hopper.set(targetHopperSpeed);
        }
    }

    /**
     * Gets the predicted hopper fullness amount 
     * using the hopper limelight.
     * <p>
     * Yes, real term
     * 
     * @return The hopper fullness amount
     */
    public double getPredictedHopperFullnessAmount() {
        return HopperFullnessAmountByTA.get(hopperLLTA);
    }

    /**
	 * Sets the state of the hopper.
	 * <p> 
	 * If wanting to control the hopper without PID
	 * then use setHopperDumbControl()
	 * 
	 * @param stateToChangeTo Using HopperConstants.State
	 */
    public void setHopperState(State stateToChangeTo) {
        state = stateToChangeTo;
    }

	/**
	 * Sets the state of the hopper to DumbControl
	 * <p>
	 * Used if want to control the hopper open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param beltsSpeedInPercent The speed to control the belts
	 * 						      motor in percent
     * 
     * @param hopperSpeedInPercent The speed to control the hopper
	 * 						       motor in percent
	 */
    public void setHopperDumbControl(double beltsSpeedInPercent, double hopperSpeedInPercent) {
        targetBeltsSpeed = beltsSpeedInPercent;
        targetHopperSpeed = hopperSpeedInPercent;
        state = State.DumbControl;
    }
}
