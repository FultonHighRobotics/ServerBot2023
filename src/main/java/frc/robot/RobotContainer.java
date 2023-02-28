package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.FollowTrajectory;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.commands.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.drivebase.TeleopDrive;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  
  XboxController driverXbox = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase, () -> (Math.abs(driverXbox.getLeftY()) > OperatorConstants.LEFT_Y_DEADBAND) ? driverXbox.getLeftY() : 0, () -> (Math.abs(driverXbox.getLeftX()) > OperatorConstants.LEFT_X_DEADBAND) ? driverXbox.getLeftX() : 0, () -> -driverXbox.getRightX(), () -> driverXbox.getRightY(), false);
    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase, () -> (Math.abs(driverXbox.getLeftY()) > OperatorConstants.LEFT_Y_DEADBAND) ? driverXbox.getLeftY() : 0, () -> (Math.abs(driverXbox.getLeftX()) > OperatorConstants.LEFT_X_DEADBAND) ? driverXbox.getLeftX() : 0, () -> driverXbox.getRightY(), false);
    TeleopDrive closedFieldRel = new TeleopDrive(drivebase, () -> (Math.abs(driverXbox.getLeftY()) > OperatorConstants.LEFT_Y_DEADBAND) ? driverXbox.getLeftY() : 0, () -> (Math.abs(driverXbox.getLeftX()) > OperatorConstants.LEFT_X_DEADBAND) ? driverXbox.getLeftX() : 0, () -> -driverXbox.getRightX(), () -> true, false);

    //drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedFieldAbsoluteDrive : closedAbsoluteDrive);
    drivebase.setDefaultCommand(closedFieldRel);
  }

  private void configureBindings() {
    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return Autos.exampleAuto(drivebase);
    
  }

  public void setDriveMode(){
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake){
    drivebase.setMotorBrake(brake);
  }

}
