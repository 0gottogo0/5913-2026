// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.FeederConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    private TalonFX feeder = new TalonFX(MotorID);
    private TalonFXConfiguration feederConfig = new TalonFXConfiguration();

	private VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0);

    private double targetSpeed = 0;

    public State state = State.Idle;

    public Feeder() {
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  	  	feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		feederConfig.Slot0.kV = PIDkV;
		feederConfig.Slot0.kP = PIDkP;
		feederConfig.Slot0.kI = PIDkI;
		feederConfig.Slot0.kD = PIDkD;


  	  	feeder.clearStickyFaults();
  	  	feeder.getConfigurator().apply(feederConfig);
    }

    @Override
    public void periodic() {
        if (state == State.Idle) {
            feeder.set(0.00);
        } else if (state == State.Feed) {
            feeder.setControl(feederVelocityVoltage.withVelocity(targetSpeed));
        } else if (state == State.Unstick) {
            feeder.setControl(feederVelocityVoltage.withVelocity(UnstickSpeed));
        } else if (state == State.Outtake) {
            feeder.setControl(feederVelocityVoltage.withVelocity(OuttakeSpeed));
        } else {
            feeder.set(targetSpeed);
        }

        SmartDashboard.putNumber("Feeder RPS", feeder.getVelocity().getValueAsDouble());

        SmartDashboard.putString("Feeder State", state.toString());
    }

    /**
	 * Sets the state of the feeder.
	 * <p> 
	 * If wanting to control the feeder without PID
	 * then use setFeederDumbControl()
	 * 
	 * @param stateToChangeTo Using FeederConstants.State	
     * @param speedInRPS The target speed to set in rps.
     *                   The feed ratio is applied later so
     *                   doesnt need to be used when determining
	 * 					 speed. When setting the state other
     *                   then feed, the target speed is not
     *                   used and can be set to 0.
	 */
    public void setFeederState(State stateToChangeTo, double speedInRPS) {
        targetSpeed = speedInRPS * FeedingRatio;
        state = stateToChangeTo;
    }

	/**
	 * Sets the state of the feeder to DumbControl
	 * <p>
	 * Used if want to control the feeder open loop without
	 * the PID. Uses the TalonFX .set() function 
	 * 
	 * @param speedInPercent The speed to control the feeder
	 * 						 motor in percent
	 */
    public void setFeederDumbControl(double speedInPercent) {
        targetSpeed = speedInPercent;
        state = State.DumbControl;
    }
}
