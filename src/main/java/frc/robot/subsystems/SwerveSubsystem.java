// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveKinematics2;


public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;


  /** Creates a new SwerveDrive. */
  public SwerveSubsystem(File directory) {
    try{
      swerveDrive = new SwerveParser(directory).createSwerveDrive();
    } catch (Exception e){
      throw new RuntimeException(e);
    }
  }

  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg){
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
  }

  @Override
  public void simulationPeriodic(){}
 
  public SwerveKinematics2 getKinematics(){
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory){
    swerveDrive.postTrajectory(trajectory);
  }

  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }

  public void setMotorBrake(boolean brake){
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading(){
    return swerveDrive.getYaw();
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY){
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle){
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians());
  }

  public ChassisSpeeds getFieldVelocity(){
    return swerveDrive.getFieldVelocity();
  }

  public SwerveController getSwerveController(){
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration(){
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock(){
    swerveDrive.lockPose();
  }

  public void addFakeVisionReading(){
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp(), false);
  }

}
