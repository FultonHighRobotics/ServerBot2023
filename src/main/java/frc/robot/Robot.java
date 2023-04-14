package frc.robot;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Arm.ArmPosition;
import swervelib.parser.SwerveParser;


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
    CameraServer.startAutomaticCapture();

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
    String chosen = m_robotContainer.autoCooser.getSelected();
    m_robotContainer.setMotorBrake(true);
    arm.MoveToPosition(ArmPosition.extended);
    arm.MoveIntake(1);
    arm.MoveToPosition(ArmPosition.retracted);
    if (chosen == "n") return;

    if (chosen == "f")
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if(m_autonomousCommand != null){
      m_autonomousCommand.schedule();
    }
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    arm.MoveIntake(0);
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
  }

  /** This function is called periodically during operator control. */
  boolean upR;
  boolean downR;
  @Override
  public void teleopPeriodic() {
    arm.SetIntake(m_robotContainer.driverXbox.getLeftBumper(), m_robotContainer.driverXbox.getRightBumper());
    arm.SetArm(m_robotContainer.driverXbox.getRawAxis(2), m_robotContainer.driverXbox.getRawAxis(3));

    if (m_robotContainer.driverXbox.getYButtonPressed()){
      if (m_robotContainer.power == 1)
        m_robotContainer.power = 0.5;
      else
        m_robotContainer.power = 1;
    }

    SmartDashboard.putNumber("power", m_robotContainer.power);
    if (m_robotContainer.driverXbox.getPOV() == 0){
      if (upR){
        if(m_robotContainer.power < 1.2)
          m_robotContainer.power += 0.05;
        upR = false;
      }
    }
    else
      upR = true;

    SmartDashboard.putNumber("pov",m_robotContainer.driverXbox.getPOV());
    if (m_robotContainer.driverXbox.getPOV() == 180){
      if (downR){
        if (m_robotContainer.power > 0.35)
          m_robotContainer.power -= 0.05;
        downR = false;
      }
    }
    else
      downR = true;

    if (m_robotContainer.driverXbox.getBButtonPressed()){
        arm.rollerEnabled = !arm.rollerEnabled;
    }
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
