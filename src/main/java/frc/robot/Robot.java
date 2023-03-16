// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static Robot instance;
  private Command m_autonomousCommand;
  public RobotContainer m_robotContainer;
  public Arm arm = new Arm();

  private Timer disabledTimer;
  
  public Robot(){
    instance = this;
  }

  public static Robot getInstance(){
    return instance;
  }


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    disabledTimer = new Timer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if(disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME)){
      m_robotContainer.setMotorBrake(false);
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if(m_autonomousCommand != null){
      m_autonomousCommand.schedule();
    }
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
  }

  /** This function is called periodically during operator control. */
  boolean released;
  @Override
  public void teleopPeriodic() {
    arm.SetIntake(m_robotContainer.driverXbox.getLeftBumper(), m_robotContainer.driverXbox.getRightBumper());
    arm.SetArm(m_robotContainer.driverXbox.getRawAxis(2), m_robotContainer.driverXbox.getRawAxis(3));;
    
    if (m_robotContainer.driverXbox.getRawButton(1)){
      if (released){
        released = false;
        arm.rollerEnabled = !arm.rollerEnabled;
      }
    }
    else
      released = true;
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    try{
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    }catch(IOException e){
      throw new RuntimeException(e);
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
