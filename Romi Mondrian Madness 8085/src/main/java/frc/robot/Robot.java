// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final RomiDrivetrain m_drivetrain = new RomiDrivetrain();
  public final Joystick ps4controller = new Joystick(0);
  public final Timer m_Timer = new Timer();
  public final SlewRateLimiter filter = new SlewRateLimiter(0.5);

  // keeps track of what Autonomous State we are in 
  private int autoState;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    m_drivetrain.resetEncoders();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
      
      break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // robot drive by the input of a controller (joystick)
    double speed = -0.9*ps4controller.getY();
    double turn = 0.75*ps4controller.getZ();   
    
    // if we were speeding up, use limitedSpeed otherwise use speed
    // What is speeding up?
    double limitedSpeed;
    
    if (Math.abs(speed) > 0.1){
      limitedSpeed = filter.calculate(speed);
    }
    else{
      limitedSpeed = speed;
    }

    limitedSpeed = deadBandHandler(limitedSpeed);
    turn = deadBandHandler(turn);

    m_drivetrain.arcadeDrive(limitedSpeed,turn);
  } 


  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    autoState = 1;
    m_drivetrain.resetEncoders();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
      // Value is Squared, so compensating
      double tempSpeed;
      double turnRight;
      tempSpeed = Math.sqrt(.35);
      turnRight = Math.sqrt(.35);
      
      if ((m_drivetrain.getLeftDistanceInch() < 14.5 ) && (autoState==1) ) {
        m_drivetrain.arcadeDrive(tempSpeed, 0);
      }
      else if ((m_drivetrain.getLeftDistanceInch()) > 14.5 && (autoState==1)) {
        m_drivetrain.arcadeDrive(0, 0);
        autoState=2;
        m_drivetrain.resetEncoders();
      }
      else if ((Math.abs(m_drivetrain.getRightDistanceInch()) < 4 ) && (autoState==2) ) {
        m_drivetrain.arcadeDrive(0,-turnRight);
      }
      else if ((m_drivetrain.getRightDistanceInch()) > 4 && (autoState==2)) {
        m_drivetrain.arcadeDrive(0, 0);
        autoState=3;
        m_drivetrain.resetEncoders();
      }
      else if ((m_drivetrain.getLeftDistanceInch() < 14 ) && (autoState==3) ) {
        m_drivetrain.arcadeDrive(tempSpeed, 0);
      }
      else if ((m_drivetrain.getLeftDistanceInch()) > 14 && (autoState==3)) {
        m_drivetrain.arcadeDrive(0, 0);
        autoState=4;
        m_drivetrain.resetEncoders();
      }
      else if ((Math.abs(m_drivetrain.getRightDistanceInch()) < 4.5 ) && (autoState==4) ) {
        m_drivetrain.arcadeDrive(0,-turnRight);
      }
      else if ((m_drivetrain.getRightDistanceInch()) > 4.5 && (autoState==4)) {
        m_drivetrain.arcadeDrive(0, 0);
        autoState=5;
        m_drivetrain.resetEncoders();
      }
      else if ((m_drivetrain.getLeftDistanceInch() < 24 ) && (autoState==5) ) {
        m_drivetrain.arcadeDrive(tempSpeed, (-(Math.sqrt(.04))));
      }
      else if ((m_drivetrain.getLeftDistanceInch()) > 24 && (autoState==5)) {
        m_drivetrain.arcadeDrive(0, 0);
        autoState=6;
        m_drivetrain.resetEncoders();
      }
      else if ((Math.abs(m_drivetrain.getLeftDistanceInch()) < 6 ) && (autoState==6) ) {
        m_drivetrain.arcadeDrive(0,turnRight);
      }
      else if ((m_drivetrain.getLeftDistanceInch()) > 6 && (autoState==6)) {
        m_drivetrain.arcadeDrive(0, 0);
        autoState=7;
        m_drivetrain.resetEncoders();
      }
      else if ((m_drivetrain.getLeftDistanceInch() < 18.5 ) && (autoState==7) ) {
        m_drivetrain.arcadeDrive(tempSpeed, 0);
      }
      else if ((m_drivetrain.getLeftDistanceInch()) > 18.5 && (autoState==7)) {
        m_drivetrain.arcadeDrive(0, 0);
        autoState=8;
        m_drivetrain.resetEncoders();
      }
      else if ((Math.abs(m_drivetrain.getLeftDistanceInch()) < 4.25 ) && (autoState==8) ) {
        m_drivetrain.arcadeDrive(0,turnRight);
      }
      else if ((m_drivetrain.getLeftDistanceInch()) > 4.25 && (autoState==8)) {
        m_drivetrain.arcadeDrive(0, 0);
        autoState=9;
        m_drivetrain.resetEncoders();
      }
      else if ((m_drivetrain.getLeftDistanceInch() < 16 ) && (autoState==9) ) {
        m_drivetrain.arcadeDrive(tempSpeed, 0);
      }
      else if ((m_drivetrain.getLeftDistanceInch()) > 16 && (autoState==9)) {
        m_drivetrain.arcadeDrive(0, 0);
        autoState=10;
        m_drivetrain.resetEncoders();
      }
      else {
        m_drivetrain.arcadeDrive(0, 0);
      }

  }

  public double deadBandHandler(double input) {
    //If input is greater than a certain value, then use that input, otherwise use 0
    double deadBandValue = 0.05;
    if(Math.abs(input) > deadBandValue) {
      return input;
    }
    else {
      return 0; 
    }
  }

}
