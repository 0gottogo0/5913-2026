// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.ClimberConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private TalonFX elevator = new TalonFX(MotorID); // create constants and change motor names
    private TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

	private VelocityVoltage elevatorVelocityVoltage = new VelocityVoltage(0);

    private double targetSpeed = 0;

    public State state = State.Idle;

    public Climber() {
        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  	  	elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		elevatorConfig.Slot0.kV = PIDkV;
		elevatorConfig.Slot0.kP = PIDkP;
		elevatorConfig.Slot0.kI = PIDkI;
		elevatorConfig.Slot0.kD = PIDkD;


  	  	elevator.clearStickyFaults();
  	  	elevator.getConfigurator().apply(elevatorConfig);
    }

    @Override
    public void periodic() {
        if (state == State.Idle) {
            elevator.set(0);
        } else if (state == State.DumbControl) {
            elevator.set(targetSpeed);
        }
    }

    /**
	 * Sets the state of the elevator.
	 * <p> 
	 * If wanting to control the elevator without PID
	 * then use setElevatorDumbControl()
	 * 
	 * @param stateToChangeTo Using elevatorConstants.State
	 * @param temp Not implimented yet
	 */
	public void setElevatorState(State stateToChangeTo, double temp) {
		targetSpeed = temp;
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
