// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ControlSub;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    ControlSub control = new ControlSub();

    public RobotContainer() {
        NamedCommands.registerCommand("Stop Intake Out", control.runOnce(
            () -> control.stopIntakeOut()
        ));

        NamedCommands.registerCommand("Stop Intake In", control.runOnce(
            () -> control.stopIntakeIn()
        ));

        NamedCommands.registerCommand("Run Intake Out", control.runOnce(
            () -> control.startIntakeOut()
        ));

        NamedCommands.registerCommand("Run Intake In", control.runOnce(
            () -> control.startIntakeIn()
        ));

        NamedCommands.registerCommand("Stop Spinup", control.runOnce(
            () -> control.stopSpinup()
        ));

        NamedCommands.registerCommand("Start Spinup", control.runOnce(
            () -> control.startSpinup()
        ));

        NamedCommands.registerCommand("Stop Shooting", control.runOnce(
            () -> control.stopShooting()
        ));

        NamedCommands.registerCommand("Start Shooting", control.runOnce(
            () -> control.startShooting()
        ));

        NamedCommands.registerCommand("Stop Tracking", control.runOnce(
            () -> control.stopTracking()
        ));

        NamedCommands.registerCommand("Start Tracking", control.runOnce(
            () -> control.startTracking()
        ));

        configureBindings();

        control.drivetrain.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
