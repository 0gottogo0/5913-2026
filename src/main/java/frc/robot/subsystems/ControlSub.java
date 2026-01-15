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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;

public class ControlSub extends SubsystemBase {

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    Shooter shooter = new Shooter();
    Feeder feeder = new Feeder();
    
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
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-DriverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-DriverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-DriverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        if (DriverStation.isDisabled()) {
            drivetrain.applyRequest(() -> idle).ignoringDisable(true);
        }

        if (DriverController.button(6).getAsBoolean()) {
            drivetrain.seedFieldCentric();
        }

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

        // "Reset" button
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
        driverLastB = DriverController.b().getAsBoolean();
        driverLastX = DriverController.x().getAsBoolean();
        driverLastY = DriverController.y().getAsBoolean();
        driverLastPovUp = DriverController.povUp().getAsBoolean();
        driverLastPovDown = DriverController.povDown().getAsBoolean();
    }
}
