// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  	private TalonFX intake = new TalonFX(MotorID);
  	private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

	private VelocityVoltage intakeVelocityVoltage = new VelocityVoltage(0);

  	private double targetSpeed = 0;

	public State state = State.Idle;

  	public Intake() {
		intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		intakeConfig.Slot0.kV = PIDkV;
		intakeConfig.Slot0.kP = PIDkP;
		intakeConfig.Slot0.kI = PIDkI;
		intakeConfig.Slot0.kD = PIDkD;

  	  	intake.clearStickyFaults();
  	  	intake.getConfigurator().apply(intakeConfig);
  	}

  	@Override
  	public void periodic() {
		if (state == State.Idle) {
			intake.set(0.00);
		} else if (state == State.Intake) {
			intake.setControl(intakeVelocityVoltage.withVelocity(IntakingSpeed));
		} else if (state == State.Unstick) {
			intake.setControl(intakeVelocityVoltage.withVelocity(OuttakingSpeed));
		} else if (state == State.Outtake){
			intake.setControl(intakeVelocityVoltage.withVelocity(OuttakingSpeed));
		} else {
			intake.set(targetSpeed);
		}

        SmartDashboard.putNumber("Intake RPS", intake.getVelocity().getValueAsDouble());

        SmartDashboard.putString("Intake State", state.toString());
  	}

	/**
	 * Sets the state of the intake.
	 * <p> 
	 * If wanting to control the intake without PID
	 * then use setIntakeDumbControl()
	 * 
	 * @param stateToChangeTo Using IntakeConstants.State
	 */
	public void setIntakeState(State stateToChangeTo) {
		state = stateToChangeTo;
	}

	/**
	 * Sets the state of the intake to DumbControl
	 * <p>
	 * Used if want to control the intake open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param speedInPercent The speed to control the intake
	 * 						 motor in percent
	 */
	public void setIntakeDumbControl(double speedInPercent) {
		targetSpeed = speedInPercent;
		state = State.DumbControl;
	}
}


