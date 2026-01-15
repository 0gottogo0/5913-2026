// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControlSub extends SubsystemBase {
    
    Shooter shooter = new Shooter();
    
    private final CommandXboxController DriverController = new CommandXboxController(1);
    private final CommandXboxController ManipulatorController = new CommandXboxController(2);

    private boolean driverLastA = DriverController.a().getAsBoolean();
    private boolean driverLastB = DriverController.b().getAsBoolean();
    private boolean driverLastX = DriverController.x().getAsBoolean();
    private boolean driverLastY = DriverController.y().getAsBoolean();
    private boolean driverLastPovUp = DriverController.povUp().getAsBoolean();
    private boolean driverLastPovDown = DriverController.povDown().getAsBoolean();

    private double shooterSpeed = 0;

    public ControlSub() {

    }

    @Override
    public void periodic() {
        // Check for input then call subsystems

        if (!driverLastA && DriverController.a().getAsBoolean()) {
            --shooterSpeed;
        }

        if (!driverLastY && DriverController.y().getAsBoolean()) {
            --shooterSpeed;
        }

        // Inputs are now "outdated" and can be compared with new ones next cycle
        driverLastA = DriverController.a().getAsBoolean();
        driverLastB = DriverController.b().getAsBoolean();
        driverLastX = DriverController.x().getAsBoolean();
        driverLastY = DriverController.y().getAsBoolean();
        driverLastPovUp = DriverController.povUp().getAsBoolean();
        driverLastPovDown = DriverController.povDown().getAsBoolean();
    }
}
