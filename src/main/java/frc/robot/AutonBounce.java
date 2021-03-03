package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonBounce extends Auton {

    //Initalizes the Auton Steps
    private int autonIndex = 1;
    private CurvedDrive autonStep1;
    private BottleTurn autonStep2;
    private CurvedDrive autonStep3;
    private BottleTurn autonStep4;
    private CurvedDrive autonStep5;
    private BottleTurn autonStep6;
    private CurvedDrive autonStep7;


    public AutonBounce(SwerveDrive drive) {

        //Creates the Auton Steps
        super(drive);
        autonIndex = 1;
        autonStep1 = new CurvedDrive(drive, 4, new double[][] {{40.0,-90.0,-45.0},{40,-180,-45},{40,0,-45},{10,-45,-45}});
        autonStep2 = new BottleTurn(drive, 90, false);
        autonStep3 = new CurvedDrive(drive, 2, new double[][] {{60.0,-165.0,0.0},{70.0,15.0,-90.0}});
        autonStep4 = new BottleTurn(drive, -180, false);
        autonStep5 = new CurvedDrive(drive, 1, new double[][] {{25.0,-85.0,-180.0}});
        autonStep6 = new BottleTurn(drive, 90, false);
        autonStep7 = new CurvedDrive(drive, 3, new double[][] {{70.0,-180.0,90.0},{50.0,0.0,90.0},{20.0,-90.0,90.0}});

        //SmartDashboard. putNumber("Auton Status", 0);
    }

    public void run(){
        //SmartDashboard.putNumber("Auton Index", autonIndex);

        //Runs the current Auton Step
        if(autonIndex == 1){
          if(autonStep1.curvedDrive()){
            autonIndex++;
          }
        }else if(autonIndex == 2){
          if(autonStep2.bottleTurn()){
            autonIndex++;
          }
        }else if(autonIndex == 3){
          if(autonStep3.curvedDrive()){
            autonIndex++;
          }
        }else if(autonIndex == 4){
          if(autonStep4.bottleTurn()){
            autonIndex++;
          }
        }else if(autonIndex == 5){
          if(autonStep5.curvedDrive()){
            autonIndex++;
          }
        }else if(autonIndex == 6){
          if(autonStep6.bottleTurn()){
            autonIndex++;
          }
        }else if(autonIndex == 7){
            if(autonStep7.curvedDrive()){
              autonIndex++;
            }
        }else{
            super.drive.stopSwerveDrive();
            //SmartDashboard. putNumber("Auton Status", 1);
        }
    }
}
