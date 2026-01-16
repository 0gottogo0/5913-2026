// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.TunerConstants;

public class ControlSub extends SubsystemBase {

    private double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    Feeder feeder = new Feeder();
    Shooter shooter = new Shooter();
    
    private final CommandXboxController DriverController = new CommandXboxController(0);
    private final CommandXboxController ManipulatorController = new CommandXboxController(1);

    private boolean driverLastA = DriverController.a().getAsBoolean();
    private boolean driverLastY = DriverController.y().getAsBoolean();
    private boolean driverLastPovUp = DriverController.povUp().getAsBoolean();
    private boolean driverLastPovDown = DriverController.povDown().getAsBoolean();

    private double shooterSpeed = 0;

    public ControlSub() {
    
    }

    @Override
    public void periodic() {
        // Check for input then call subsystems

        /* Driver Controls */
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-DriverController.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
                 .withVelocityY(-DriverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
                 .withRotationalRate(-DriverController.getRightX() * maxAngularRate) // Drive counterclockwise with negative X (left)
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        if (DriverStation.isDisabled()) {
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true);
        }

        if (DriverController.button(7).getAsBoolean()) {
            drivetrain.seedFieldCentric();
        }

        if (DriverController.button(8).getAsBoolean()) {
            drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
        }

        // temp buttons
        if (!driverLastA && DriverController.a().getAsBoolean()) {
            shooterSpeed = shooterSpeed - 10;
        }

        if (!driverLastY && DriverController.y().getAsBoolean()) {
            shooterSpeed = shooterSpeed + 10;
        }

        if (!driverLastPovDown && DriverController.povDown().getAsBoolean()) {
            shooterSpeed = shooterSpeed - 100;
        }

        if (!driverLastPovUp && DriverController.povUp().getAsBoolean()) {
            shooterSpeed = shooterSpeed + 100;
        }

        // "reset" button
        if (DriverController.b().getAsBoolean()) {
            shooter.stopShooting();
            shooterSpeed = 0;
        } else {
            shooter.startShooting(shooterSpeed);
        }

        if (DriverController.x().getAsBoolean()) {
            feeder.startFeeder();
        } else {
            feeder.stopFeeder();
        }

        SmartDashboard.putNumber("Shooter Target Speed", shooterSpeed);

        // Inputs are now "outdated" and can be compared with new ones next scheduler run
        driverLastA = DriverController.a().getAsBoolean();
        driverLastY = DriverController.y().getAsBoolean();
        driverLastPovUp = DriverController.povUp().getAsBoolean();
        driverLastPovDown = DriverController.povDown().getAsBoolean();
    }
}
