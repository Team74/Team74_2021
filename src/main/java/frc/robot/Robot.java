// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private double driveAngle;
  DigitalInput testSensor;
  XboxController driverController;
  CANSparkMax testMotor;
  AHRS gyro;
  Ma3Encoder testEncoder;
  SwerveModule testSwerve;
  SwerveDrive drive;
  SwerveModule[] module;
  public Robot() {
    //super(0.01);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    //testSensor = new DigitalInput(2);
    driverController = new XboxController(0);
    //testMotor = new CANSparkMax(3,MotorType.kBrushless);
    gyro = new AHRS();
    //testSwerve = new SwerveModule(3,2,2,58.0);
    drive = new SwerveDrive(module = new SwerveModule[]{     //Drive Motor, Rotation Motor, Rotation Encoder, Rotation Offset
      new SwerveModule(5,14,0,-135.79),           //Front Left
      new SwerveModule(12,15,1,-168.49),           //Front Right
      new SwerveModule(11,3,2,-1.58),           //Back Right
      new SwerveModule(44,4,3,60.03)            //Back Left
    });
    SmartDashboard.setDefaultNumber("PID p",1.0);
    SmartDashboard.setDefaultNumber("PID d",0.05);
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
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    double PIDp = SmartDashboard.getNumber("PID p", 1.0);
    double PIDd = SmartDashboard.getNumber("PID d", 0.05);
    //testSwerve.SetPIDParameters(PIDp, PIDd);
  }
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double[] angles = new double[4];
    angles = drive.getAngle();

    for(int index = 0; index<4; index++){
        SmartDashboard.putNumber("Angle " + index, angles[index]);
    }

    //float testFloat = gyro.getYaw();
    //System.out.println(testFloat);
    //double angle = testEncoder.getAngle();
    double controllerDeadzone = 0.1;
    double driveControllerLeftY = -1*driverController.getY(Hand.kLeft);
    double driveControllerLeftX = driverController.getX(Hand.kLeft);
    double driveControllerRightX = -1*driverController.getX(Hand.kRight);
    if(Math.abs(driveControllerLeftY)<controllerDeadzone){
      driveControllerLeftY=0;
    }
    if(Math.abs(driveControllerLeftX)<controllerDeadzone){
      driveControllerLeftX=0;
    }
    if(Math.abs(driveControllerRightX)<controllerDeadzone){
      driveControllerRightX=0;
    }
    ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(
      driveControllerLeftY,
      driveControllerLeftX,
      driveControllerRightX,
      new Rotation2d(gyro.getAngle()*Math.PI/180)
    );
    drive.MoveSwerveDrive(speed);

    for(int index = 0; index<4; index++){
        SmartDashboard.putNumber("Swerve Module" + index, module[index].getAngle());
    }

    if(driverController.getRawButtonPressed(4)){
      gyro.reset();
      System.out.println("gyro reset");
    }
    
    //SmartDashboard.putNumber("Controller X", driveControllerRightX);
    //SmartDashboard.putNumber("Controller Y", driveControllerRightY);
    //SmartDashboard.putNumber("Target Angle", driveAngle);
    
    //System.out.println();
    //double testValue = driverController.getY(Hand.kRight);
    //System.out.println(testValue);
    //testMotor.set(testValue);
    /*boolean testValue = driverController.getRawButton(1);
    if(testValue){
      System.out.println("yes");
    }else{
      System.out.println("no");
    }
    */
  // hello world
  // another comment
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
