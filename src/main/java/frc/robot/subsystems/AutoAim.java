// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.Constants.AutoAimConstants.BlueGoal;
import static frc.robot.constants.Constants.AutoAimConstants.RedGoal;
import static frc.robot.constants.Constants.AutoAimConstants.TimeOfFlightByDistance;
import static frc.robot.constants.Constants.AutoAimConstants.TurretRotatePoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoAim extends SubsystemBase {

    Pose2d robotPose = new Pose2d();
    ChassisSpeeds robotSpeed = new ChassisSpeeds();

    Pose2d TurretRotatePointPose = new Pose2d();

    Pose2d goalPose = new Pose2d();
    Pose2d adjustedGoalPose = new Pose2d();

    double calculatedShot[] = {0.00, 0.00, 0.00};

    public AutoAim() {
        
    }

    @Override
    public void periodic() {
        try {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                goalPose = BlueGoal;
            } else {
                goalPose = RedGoal;
            }
        } catch (Exception e) {
            goalPose = RedGoal;
        }

        adjustedGoalPose = goalPose.plus(new Transform2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond, new Rotation2d()).times(TimeOfFlightByDistance.get(getAimTargetInDistance())));

        TurretRotatePointPose = robotPose.plus(TurretRotatePoint);

        SmartDashboard.putNumber("Robot X", robotPose.getX());
        SmartDashboard.putNumber("Robot Y", robotPose.getY());
        SmartDashboard.putNumber("Robot Rot", robotPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot Speed X", robotSpeed.vxMetersPerSecond);
        SmartDashboard.putNumber("Robot Speed Y", robotSpeed.vyMetersPerSecond);
        SmartDashboard.putNumber("Robot Speed Rot", Math.toDegrees(robotSpeed.omegaRadiansPerSecond));
        SmartDashboard.putNumber("Turret X", TurretRotatePointPose.getX());
        SmartDashboard.putNumber("Turret Y", TurretRotatePointPose.getY());

        SmartDashboard.putNumber("Target Angle", getAimTargetInDegrees());
        SmartDashboard.putNumber("Target Distance", getAimTargetInDistance());
        SmartDashboard.putNumber("Target Time Of Flight", TimeOfFlightByDistance.get(getAimTargetInDistance()));
        
        SmartDashboard.putNumber("Adjusted Target Angle", getShootOnMoveAimTarget()[0]);
        SmartDashboard.putNumber("Adjusted Target Speed", getShootOnMoveAimTarget()[1]);
        SmartDashboard.putNumber("Adjusted Target Time Of Flight", getShootOnMoveAimTarget()[2]);
    }

    /**
     * Give the auto aim subsystem the robots state
     * 
     * @param drivetrainPos The robots position
     * @param drivetrainSpeed The robots speed
     */
    public void setAutoAimDrivetrainState(Pose2d drivetrainPos, ChassisSpeeds drivetrainSpeed) {
        robotPose = drivetrainPos;
        robotSpeed = drivetrainSpeed;
    }

    /**
     * Get angle from robot to target
     * 
     * @return Aim tagret in degrees
     */
    private double getAimTargetInDegrees() {
        return robotPose.getRotation().getDegrees() - Math.atan(TurretRotatePointPose.minus(goalPose).getX() / TurretRotatePointPose.minus(goalPose).getY());
    }
    
    /**
     * Get distance from robot to target
     * 
     * @return Aim target in meters
     */
    private double getAimTargetInDistance() {
        return Math.abs(Math.sqrt(Math.pow(TurretRotatePointPose.minus(goalPose).getX(), 2) + Math.pow(TurretRotatePointPose.minus(goalPose).getY(), 2)));
    }

    /**
     * Get calculated angle, speed, and time of
     * flight to shoot into the target on the
     * move
     * 
     * @return An array with degrees, rps, and
     *         secconds
     */
    public double[] getShootOnMoveAimTarget() {
        calculatedShot[0] = robotPose.getRotation().getDegrees() - Math.atan(TurretRotatePointPose.minus(adjustedGoalPose).getX() / TurretRotatePointPose.minus(adjustedGoalPose).getY());
        calculatedShot[1] = Math.abs(Math.sqrt(Math.pow(TurretRotatePointPose.minus(adjustedGoalPose).getX(), 2) + Math.pow(TurretRotatePointPose.minus(adjustedGoalPose).getY(), 2)));
        calculatedShot[2] = TimeOfFlightByDistance.get(calculatedShot[1]);
        return calculatedShot;
    }
}
