package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  RelativeEncoder armEncoder;

  CANSparkMax arm;
  CANSparkMax intake;

  public boolean rollerEnabled;

  // constants
  static final int ARM_CURRENT_LIMIT_A = 20;
  static final double ARM_OUTPUT_POWER = 0.65;
  static final int intakeLimitAmps= 25;
  static final int intakeHoldAmps = 5;
  static final double intakeOutputPower = 1.0;
  static final double intakeHoldPower = 0.04;
  static final double ARM_HOLD_POWER = 0.05;

  double armBackEncPos = 380;
  double armForwardEncPos = -860;
  double armInclineEncPos = -12;


  public enum ArmPosition{
    extended,
    middle,
    retracted
  }


  public void MoveToPosition(ArmPosition pos){
    if(pos == ArmPosition.retracted){
      while(armEncoder.getPosition() < armBackEncPos){
        setArmMotor(0.4);
      } 
    }
    if(pos == ArmPosition.extended){
      while(armEncoder.getPosition() > armForwardEncPos){
        setArmMotor(-0.4);
      }
    }
    setArmMotor(0);
  }

  public int time = 0;
  public boolean the = true;
  public void MoveIntake(int d){
    if (time <= 0 && the){
      setIntakeMotor(d * intakeOutputPower, intakeLimitAmps);
      time = 3;
      the = false;
    }
    else
      time--;
    SmartDashboard.putNumber("69", time);
  }

  public Arm(){
    intake = new CANSparkMax(14, MotorType.kBrushless);
    arm = new CANSparkMax(15, MotorType.kBrushless);

    armEncoder = arm.getEncoder();

    arm.setInverted(true);
    arm.setIdleMode(IdleMode.kBrake);
    arm.setSmartCurrentLimit(ARM_CURRENT_LIMIT_A);
    intake.setInverted(false);
    intake.setIdleMode(IdleMode.kBrake);

  }

  public double clamp(double value, double min, double max){
    if (value > min && value < max)
      return value;
    else if (value > max)
      return max;
    else
      return min;
  }

  public void setEncoderPositions(double backward){
    armBackEncPos = backward;
    armForwardEncPos = backward - 41.8;
    armInclineEncPos = backward - 17.4;
  }

  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  public enum direction{
    up,
    down
  }


  public void setArm(int pos){
    arm.set(0.4);
    
  }

  public void setArmMotor(double percent) {
    arm.set(percent);
    //SmartDashboard.putNumber("arm power (%)", percent);
    //SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
  }

  public void setIntakeMotor(double percent, int amps) {
    intake.set(percent);
    intake.setSmartCurrentLimit(amps);
    //SmartDashboard.putNumber("intake power (%)", percent);
    //SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
    SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition());
  }

  public void SetIntake(Boolean inButton, Boolean outButton){
    double intakePower = 0;
    int intakeAmps = 0;
    if (inButton) {
      intakePower = intakeOutputPower;
      intakeAmps = intakeLimitAmps;
      lastGamePiece = CUBE;
    } else if (outButton) {
      intakePower = -intakeOutputPower;
      intakeAmps = intakeLimitAmps;
      lastGamePiece = CONE;
    } else if (lastGamePiece == CUBE && rollerEnabled) {
      intakePower = intakeHoldPower;
      intakeAmps = intakeHoldAmps;
    } else if (lastGamePiece == CONE && rollerEnabled) {
      intakePower = -intakeHoldPower;
      intakeAmps = intakeHoldAmps;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    setIntakeMotor(intakePower, intakeAmps);
  }

  direction dir = direction.down;
  public void SetArm(double a, double b){
    double armPower = 0;
    double enc = armEncoder.getPosition();
    double power = 0;

    if (a > 0.05){
        if (enc > armBackEncPos - 2)
            power = 0.3;
        else
            power = 0.6;
          
        armPower = (ARM_OUTPUT_POWER * a) * power;
        dir = direction.down;
    } 
    else if (b > 0.05){
        if (enc < armForwardEncPos + 5)
            power = 0.4;
        else
            power = 0.8;

        armPower = (-ARM_OUTPUT_POWER * b) * power;
        dir = direction.up;
    }
    else{
        if (dir == direction.up)
            armPower = -ARM_HOLD_POWER;
        else if (enc < armInclineEncPos)
            armPower = -ARM_HOLD_POWER * 0.5;
        else
            armPower = ARM_HOLD_POWER;

    }
    setArmMotor(armPower);
  }
}


	
