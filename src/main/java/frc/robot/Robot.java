package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Robot extends TimedRobot {
  PIDController pid = new PIDController(1, 0.1, 2);
  WPI_TalonFX left = new WPI_TalonFX(2); // left drive motor
  WPI_TalonFX right = new WPI_TalonFX(0); // right drive motor
  DifferentialDrive drive = new DifferentialDrive(left, right);
  ADIS16448_IMU gyro = new ADIS16448_IMU(); // RoboRIO-mounted gyroscope
  Timer timer = new Timer(); 
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0,0);
  double encoderTicksPerMeter = 2048*10.71/(0.0254*6*Math.PI); // theoretical 45812 ticks per meter
  double trackWidth = 0.69474;  // value obtained from SysID
  // motor encoder values
  double positionLeft;
  double positionRight;
  double positionAverage = (positionLeft+positionRight)/2;
  double time; // match time
  // odometry calculated robot position
  Pose2d robotPosition;
  double robotX;
  double robotY;
  double angle; // gyro angle
  double setpoint = 4;
  double error = setpoint-positionAverage;
  int autoStage = 1;

  @Override
  public void robotInit() {
    initializeMotors(); // starts and configures the motors
    timer.start(); // starts the timer at 0s.
    gyro.calibrate(); // sets the gyro angle to 0 based on the current robot position
    right.setInverted(true);
    updateVariables(); // updates and publishes variables to shuffleboard
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    updateVariables();
  }

  @Override
  public void autonomousPeriodic() {
    updateVariables();
    System.out.println("Stage");
    if (autoStage == 1) {
      drive.arcadeDrive(pid.calculate((positionLeft+positionRight)/2, setpoint), 0);
      if ((positionLeft+positionRight)/2 >= setpoint) {
        autoStage ++;
      }
    }
    /*
    if (autoStage == 2) {
      drive.arcadeDrive(0,0.5);
      if (angle >= 90) {
        autoStage ++;
      }
    }
    if (autoStage == 3) {
      drive.arcadeDrive(pid.calculate((positionLeft+positionRight)/2, 1), 0);
      if ((positionLeft+positionRight)/2 >= 5) {
        autoStage ++;
      }
    }
     */
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public void initializeMotors() {}

  // runs manufacturer recommended startup commands for Falcon 500 motors. Should be run at startup for all motors.
  public void motorConfig(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 , 30);
    motor.configNeutralDeadband(0.001, 30);
    motor.setSensorPhase(false);
    motor.setInverted(false);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
    motor.configNominalOutputForward(0, 30);
    motor.configNominalOutputReverse(0, 30);
    motor.configPeakOutputForward(1, 30);
    motor.configPeakOutputReverse(-1, 30);
    motor.selectProfileSlot(0, 0);
    motor.configOpenloopRamp(1.5);
    motor.setSelectedSensorPosition(0, 0, 30);
  }
  
  // initializes variables and publishes values on dashboard
  public void updateVariables() {
    // updates variables
    positionLeft = left.getSelectedSensorPosition(0)/encoderTicksPerMeter;
    positionRight = right.getSelectedSensorPosition(0)/encoderTicksPerMeter;
    positionAverage = (positionLeft+positionRight)/2;
    time = timer.get();
    angle = -gyro.getGyroAngleZ();
    odometry.update(new Rotation2d(angle*Math.PI/180), positionLeft, positionRight);
    robotPosition = odometry.getPoseMeters();
    robotX = robotPosition.getX();
    robotY = robotPosition.getY();
    
    // publishes updated variables to the dashboard
    SmartDashboard.putNumber("Encoder (Left)", positionLeft);
    SmartDashboard.putNumber("Encoder (Right)", positionRight);
    SmartDashboard.putNumber("Position", positionAverage);
    SmartDashboard.putNumber("Clock", time);
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("RobotX", robotX);
    SmartDashboard.putNumber("RobotY", robotY);
    SmartDashboard.putNumber("Stage", autoStage);
  }
}