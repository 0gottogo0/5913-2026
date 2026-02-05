// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.ClimberConstants.*;
import static frc.robot.constants.Constants.PneumaticConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private TalonFX pivot = new TalonFX(MotorID);
    private TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

	private PositionVoltage pivotPositionVoltage = new PositionVoltage(0);

    private double targetSpeed = 0;

    public State state = State.Idle;

    public Climber() {
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  	  	pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		pivotConfig.Slot0.kG = PIDkG;
		pivotConfig.Slot0.kP = PIDkP;
		pivotConfig.Slot0.kI = PIDkI;
		pivotConfig.Slot0.kD = PIDkD;

  	  	pivot.clearStickyFaults();
  	  	pivot.getConfigurator().apply(pivotConfig);
    }

    @Override
    public void periodic() {
        if (state == State.Idle) {
            pivot.set(0);
		} else if (state == State.ClimbUp) {
			pivot.setControl(pivotPositionVoltage.withPosition(ClimbUpSetpoint));
		} else if (state == State.ClimbDown) {
			pivot.setControl(pivotPositionVoltage.withPosition(ClimbDownSetpoint));
		} else {
            pivot.set(targetSpeed);
        }

		SmartDashboard.putNumber("Pivot Pos", pivot.getPosition().getValueAsDouble());

		SmartDashboard.putString("Climber State", state.toString());
    }

    /**
	 * Sets the state of the elevator.
	 * <p> 
	 * If wanting to control the elevator without PID
	 * then use setElevatorDumbControl()
	 * 
	 * @param stateToChangeTo Using elevatorConstants.State 
	 * 						  Note: ClimbUp refers to the
	 * 						  elevator moving down
	 */
	public void setElevatorState(State stateToChangeTo) {
		state = stateToChangeTo;
	}

    /**
	 * Sets the state of the elevator to DumbControl
	 * <p>
	 * Used if want to control the elevator open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param speedInPercent The speed to control the elevator
	 * 						 motor in percent
	 */
    public void setElevatorDumbControl(double speedInPercent) {
        targetSpeed = speedInPercent;
        state = State.DumbControl;
    }
}
