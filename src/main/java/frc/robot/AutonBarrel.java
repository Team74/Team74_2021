package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonBarrel extends Auton {
    private int autonIndex = 1;
    private CurvedDrive autonStep1;
    private BottleTurn autonStep2;
    private BottleTurn autonStep3;
    private CurvedDrive autonStep4;
    private BottleTurn autonStep5;
    private CurvedDrive autonStep6;
    private BottleTurn autonStep7;
    private CurvedDrive autonStep8;

    public AutonBarrel(SwerveDrive drive) {
        super(drive);
        autonIndex = 1;
        autonStep1 = new CurvedDrive(drive, 1, new double[][] {{77.0,-90.0,0.0}});
        autonStep2 = new BottleTurn(drive, 90, true);
        autonStep3 = new BottleTurn(drive, 0, true);
        autonStep4 = new CurvedDrive(drive, 2, new double[][] {{20.0,-75.0,-90.0},{48.0,-75.0,-180.0}});
        autonStep5 = new BottleTurn(drive, -135, false);
        autonStep6 = new CurvedDrive(drive, 1, new double[][] {{58.0,-45.0,-135.0}});
        autonStep7 = new BottleTurn(drive, 0, false);
        autonStep8 = new CurvedDrive(drive, 1, new double[][] {{170.0,90.0,0}});

        SmartDashboard. putNumber("Auton Status", 0);
    }

    public void run(){
        SmartDashboard.putNumber("Auton Index", autonIndex);
        if(autonIndex == 1){
          if(autonStep1.curvedDrive()){
            autonIndex++;
          }
        }else if(autonIndex == 2){
          if(autonStep2.bottleTurn()){
            autonIndex++;
          }
        }else if(autonIndex == 3){
          if(autonStep3.bottleTurn()){
            autonIndex++;
          }
        }else if(autonIndex == 4){
          if(autonStep4.curvedDrive()){
            autonIndex++;
          }
        }else if(autonIndex == 5){
          if(autonStep5.bottleTurn()){
            autonIndex++;
          }
        }else if(autonIndex == 6){
          if(autonStep6.curvedDrive()){
            autonIndex++;
          }
        }else if(autonIndex == 7){
            if(autonStep7.bottleTurn()){
              autonIndex++;
            }
        }else if(autonIndex == 8){
            if(autonStep8.curvedDrive()){
                autonIndex++;
            }
        }else{
            super.drive.stopSwerveDrive();
            SmartDashboard. putNumber("Auton Status", 1);
        }
    }
}
