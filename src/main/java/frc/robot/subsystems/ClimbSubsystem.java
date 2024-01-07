package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    
    private static ClimbSubsystem instance = null;

    public static ClimbSubsystem getInstance(){
        if(instance == null){
            instance = new ClimbSubsystem();
        }
        return instance;
    }
    @Override
    public void periodic(){   

    }
}
