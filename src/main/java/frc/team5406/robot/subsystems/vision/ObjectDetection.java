package frc.team5406.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetection extends SubsystemBase{
    public double foundNote = 0;
    public double noteDistance = 0;
    double frameCenter = 410;

    public void updateOrangepiTracking() {
        foundNote = Double.valueOf(NetworkTableInstance.getDefault().getTable("orangepi").getEntry("FoundNote").getString("0"));
            
        if (foundNote == 1) {
            double noteCenter = Double.valueOf(NetworkTableInstance.getDefault().getTable("orangepi").getEntry("NoteCenter").getString("0"));
            noteDistance = noteCenter-frameCenter;
        }else{
            noteDistance = 0;
        }
    }

    public double getCenterDistance(boolean note){
        double centerDistance = 0;
        if(note){
            centerDistance = noteDistance;
        }
        SmartDashboard.putNumber("oPi Center Distance", centerDistance);
        return centerDistance;
    }

    @Override
    public void periodic() {
        updateOrangepiTracking();
    }
}
