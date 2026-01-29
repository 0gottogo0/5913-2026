// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  	private TalonFX intake = new TalonFX(IntakeID);
  	private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
	private TalonFX pivot = new TalonFX(PivotID);
	private TalonFXConfiguration pivotConfig = new TalonFXConfiguration(); 

	private VelocityVoltage intakeVelocityVoltage = new VelocityVoltage(0);
	private PositionVoltage pivotPositionVoltage = new PositionVoltage(0);

  	private double targetSpeed = 0;
	private boolean pivotExtension = false;

	private Timer unstickPivotTimer = new Timer();

	public State state = State.Idle;

  	public Intake() {
		intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  	  	intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		intakeConfig.Slot0.kV = IntakePIDkV;
		intakeConfig.Slot0.kP = IntakePIDkP;
		intakeConfig.Slot0.kI = IntakePIDkI;
		intakeConfig.Slot0.kD = IntakePIDkD;

  	  	intake.clearStickyFaults();
  	  	intake.getConfigurator().apply(intakeConfig);

		pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		pivotConfig.Slot0.kG = PivotPIDkG;			
		pivotConfig.Slot0.kP = PivotPIDkP;
		pivotConfig.Slot0.kI = PivotPIDkI;
		pivotConfig.Slot0.kD = PivotPIDkD;

		pivot.clearStickyFaults();
		pivot.getConfigurator().apply(pivotConfig);
		
		unstickPivotTimer.stop();
		unstickPivotTimer.reset();
  	}

  	@Override
  	public void periodic() {
		if (state == State.Idle) {
			intake.set(0.00);
			pivot.setControl(pivotPositionVoltage.withPosition(PivotInPos));
			unstickPivotTimer.stop();
		} else if (state == State.Intake) {
			intake.setControl(intakeVelocityVoltage.withVelocity(IntakingSpeed));
			pivot.setControl(pivotPositionVoltage.withPosition(PivotOutPos));
			unstickPivotTimer.stop();
		} else if (state == State.Unstick) {
			intake.setControl(intakeVelocityVoltage.withVelocity(OuttakingSpeed));
			unstickPivotTimer.start();
		} else if (state == State.Outtake){
			intake.setControl(intakeVelocityVoltage.withVelocity(OuttakingSpeed));
			pivot.setControl(pivotPositionVoltage.withPosition(PivotOutPos));
			unstickPivotTimer.stop();
		} else {
			intake.set(targetSpeed);
			if (pivotExtension) {
				pivot.setControl(pivotPositionVoltage.withPosition(PivotOutPos));
			} else {
				pivot.setControl(pivotPositionVoltage.withPosition(PivotInPos));
			}

			unstickPivotTimer.stop();
		}

		// Cycle between pivot in and out to agitate
		// balls in hopper
		if (unstickPivotTimer.isRunning() && unstickPivotTimer.hasElapsed(1)) {
			unstickPivotTimer.reset();
			if (pivotExtension) {
				pivot.setControl(pivotPositionVoltage.withPosition(PivotInPos));
				pivotExtension = false;
			} else {
				pivot.setControl(pivotPositionVoltage.withPosition(PivotOutPos));
				pivotExtension = true;
			}
		}

        SmartDashboard.putNumber("Intake RPS", intake.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber("Intake Pivot Position", pivot.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("Intake Pivot Unstick Timer", unstickPivotTimer.get());

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
	 * @param shouldIntakeExtend Should the intake flip out
	 */
	public void setIntakeDumbControl(double speedInPercent, boolean shouldIntakeExtend) {
		targetSpeed = speedInPercent;
		pivotExtension = shouldIntakeExtend;
		state = State.DumbControl;
	}
}


