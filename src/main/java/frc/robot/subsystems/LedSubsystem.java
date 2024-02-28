package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase{
        private static final Spark ledController = new Spark(9);

        public LedSubsystem() {

        }

        public void changeColor(double color){
                ledController.set(color);
        }
    
}
