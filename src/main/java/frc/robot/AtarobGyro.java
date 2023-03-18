package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class AtarobGyro {

    public double getAngle(){
        return 0;
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(-getAngle());
    }
}
