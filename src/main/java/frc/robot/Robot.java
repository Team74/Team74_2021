// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
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
import edu.wpi.first.wpiutil.math.MathUtil;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kBounceAuto = "Bounce Auto";
  private static final String kSlalomAuto = "Slalom Auto";
  private static final String kBarrelAuto = "Barrel Auto";
  private static final String kBallAuto = "Ball Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private double driveAngle;
  private boolean driverLeftBumper = false;
  private boolean shooterHomed = true;
  XboxController driverController;
  XboxController opController;
  CANSparkMax testMotor;
  AHRS gyro;
  Ma3Encoder testEncoder;
  SwerveModule testSwerve;
  SwerveDrive drive;
  SwerveModule[] module;
  NetworkTable table;
  Auton auton;
  TalonSRX testTalon;
  Shooter shooter;
  boolean flywheelOn = false;
  double[] turretPosition;
  boolean autoTurret;

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
    m_chooser.addOption("Bounce Auto", kBounceAuto);
    m_chooser.addOption("Slalom Auto", kSlalomAuto);
    m_chooser.addOption("Barrel Auto", kBarrelAuto);
    m_chooser.addOption("Ball Auto", kBallAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    driverController = new XboxController(0);
    opController = new XboxController(1);
    //testMotor = new CANSparkMax(3,MotorType.kBrushless);
    gyro = new AHRS();
    //testSwerve = new SwerveModule(3,2,2,58.0);
    drive = new SwerveDrive(module = new SwerveModule[]{     //Drive Motor, Rotation Motor, Rotation Encoder, Rotation Offset
      new SwerveModule(5,14,0,165.76),           //Front Left
      new SwerveModule(12,15,1,175.15),           //Front Right
      new SwerveModule(11,3,2,-1.58),           //Back Right
      new SwerveModule(44,4,3,60.03)            //Back Left
    }, gyro);
    SmartDashboard.setDefaultNumber("PID p",1.0);
    SmartDashboard.setDefaultNumber("PID d",0.05);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    shooter = new Shooter(18,19,16,21);
    shooterHomed = true;
    double[] turretPosition = {0, 0};
    autoTurret = true;
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

    gyro.reset();
    switch (m_autoSelected) {
      case kBounceAuto:
        // Put custom auto code here
        auton = new AutonBounce(drive);

        break;
      case kSlalomAuto:
        // Put custom auto code here
        auton = new AutonSlalom(drive);

        break;
      case kBarrelAuto:
        // Put custom auto code here
        auton = new AutonBarrel(drive);

        break;
      case kBallAuto:
        // Put custom auto code here
        //auton = new AutonBall(drive);

        break;    
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    auton.run();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    double PIDp = SmartDashboard.getNumber("PID p", 1.0);
    double PIDd = SmartDashboard.getNumber("PID d", 0.05);
    SmartDashboard.putNumber("gyro", gyro.getAngle());
    gyro.reset();
    shooterHomed = true;
    //testSwerve.SetPIDParameters(PIDp, PIDd);

    autoTurret = false;
  }
  /** This function is called periodically during operator control. */

  @Override
  public void teleopPeriodic() {
    ChassisSpeeds speed;
    double[] angles = new double[4];
    angles = drive.getAngle();
    speed = null;
    SmartDashboard.putNumber("gyro", gyro.getAngle());

    for(int index = 0; index<4; index++){
        SmartDashboard.putNumber("Angle " + index, angles[index]);
    }

    //float testFloat = gyro.getYaw();
    //System.out.println(testFloat);
    //double angle = testEncoder.getAngle();

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    double controllerDeadzone = 0.1;
    double driveControllerLeftY = -1*driverController.getY(Hand.kLeft);
    double driveControllerLeftX = driverController.getX(Hand.kLeft);
    double driveControllerRightX = -1*driverController.getX(Hand.kRight);

    double opControllerRightTrigger = opController.getTriggerAxis(Hand.kRight);

    if(Math.abs(driveControllerLeftY)<controllerDeadzone){
      driveControllerLeftY=0;
    }
    if(Math.abs(driveControllerLeftX)<controllerDeadzone){
      driveControllerLeftX=0;
    }
    if(Math.abs(driveControllerRightX)<controllerDeadzone){
      driveControllerRightX=0;
    }
    if(!driverController.getRawButton(6)){
      driveControllerLeftY = driveControllerLeftY*0.5;
      driveControllerLeftX = driveControllerLeftX*0.5;
    }

    if(driverController.getRawButtonPressed(5)){
      driverLeftBumper = !driverLeftBumper;
    }
    /*
    if(driverController.getRawButton(2)){
      if(area>2){
        if(x>10){
          driveControllerRightX = 0.2;
        }else if(x<-10){
          driveControllerRightX = 0.4;
        }else{
          driveControllerRightX = 0.3;
        }

        driveControllerLeftX = 0.2;
      }else{
        if(x>10){
          driveControllerRightX = -0.1;
        }else if(x<-10){
          driveControllerRightX = 0.1;
        }else{
          driveControllerRightX = 0.0;
        }
      }

      if(area>6.5){
        driveControllerLeftY = -0.1;
      }else if(area<5.5){
        driveControllerLeftY = 0.1;
      }else{
        driveControllerLeftY = 0.0;
      }


      speed = new ChassisSpeeds(
        driveControllerLeftY,
        driveControllerLeftX,
        driveControllerRightX
      );
    }
    
    if(driverController.getRawButton(1)){
      if(area>2){
        if(x<-10){
          driveControllerRightX = -0.2;
        }else if(x>10){
          driveControllerRightX = -0.4;
        }else{
          driveControllerRightX = -0.3;
        }

        driveControllerLeftX = -0.2;

      }else{
        if(x>10){
          driveControllerRightX = -0.1;
        }else if(x<-10){
          driveControllerRightX = 0.1;
        }else{
          driveControllerRightX = 0.0;
        }
      }



      if(area>6.5){
        driveControllerLeftY = -0.1;
      }else if(area<5.5){
        driveControllerLeftY = 0.1;
      }else{
        driveControllerLeftY = 0.0;
      }


      speed = new ChassisSpeeds(
        driveControllerLeftY,
        driveControllerLeftX,
        driveControllerRightX
      );
    }*/

    if(driverLeftBumper){
      if(Math.abs(gyro.getAngle())>5){
        driveControllerRightX = gyro.getAngle();
        driveControllerRightX = MathUtil.clamp(driveControllerRightX, -0.5, 0.5);
      }else{
        driveControllerRightX = 0; 
      }
    }

    if(speed==null){
      speed = ChassisSpeeds.fromFieldRelativeSpeeds(
        driveControllerLeftY,
        driveControllerLeftX,
        driveControllerRightX,
        new Rotation2d(gyro.getAngle()*Math.PI/180)
      );
    }

    drive.MoveSwerveDrive(speed);

    for(int index = 0; index<4; index++){
        SmartDashboard.putNumber("Swerve Module" + index, module[index].getAngle());
    }

    if(driverController.getRawButtonPressed(4)){
      gyro.reset();
      System.out.println("gyro reset");
    }

    SmartDashboard.putNumber("Flywheel Power", opControllerRightTrigger);

    if(opController.getRawButtonPressed(8)){
      flywheelOn = !flywheelOn;
    }

    shooter.flywheelSpeed(flywheelOn);

    SmartDashboard.putNumber("Flywheel Speed", shooter.getFlywheelSpeed());

    if(opControllerRightTrigger>0.85 /*&& shooter.isFlywheelUpToSpeed(5000)*/){
      shooter.activateShooter();
    }else{
      shooter.stopShooter();
    }

    if(opController.getRawButtonPressed(7)){
      shooterHomed = false; 
    }

    if(opController.getRawButtonPressed(5)){
      autoTurret = !autoTurret;
    }


    if(shooterHomed){
      if(autoTurret){
        shooter.autoTurret();
      }else{
        shooter.manuelTurret(opController.getPOV(), opController.getRawButton(1));
      }
      SmartDashboard.putBoolean("Turret Homed", true);
    }else{
      SmartDashboard.putBoolean("Turret Homed", false);
      if(shooter.homeTurret()){
        shooterHomed = true; 
      }
    }

    double[] turretPosition = shooter.getTurretPosition();
    SmartDashboard.putNumber("Turret Pitch", turretPosition[0]);
    //SmartDashboard.putNumber("Turret Pitch", shooter.findPitch());
    SmartDashboard.putNumber("Turret Rotation Angle Tick", turretPosition[1]);
    SmartDashboard.putNumber("Turret Rotation Angle", shooter.findAngle());
    //SmartDashboard.putNumber("Turret Rotation Angle Tick Output", shooter.getRotationTick(shooter.findAngle()));

    /*if(opController.getRawButton(8)){
      shooter.flywheelSpeed(true);
    }else{
      shooter.setFlywheelSpeed(opControllerRightTrigger);
    }*/

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
