package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LedSubsystem extends SubsystemBase{
        private static final Spark ledController = new Spark(LEDConstants.channel);

        public LedSubsystem() {
        }

        public void changeColor(double color){
                ledController.set(color);
        }

        public void teamChangeColor(){
                Optional<Alliance> ally = DriverStation.getAlliance();
                if(ally.isPresent()){
                        if(ally.get() == Alliance.Red){
                                changeColor(LEDConstants.redBreathe);
                        }
                        if(ally.get() == Alliance.Blue){
                                changeColor(LEDConstants.blueBreathe);
                        }
                }
        }
    
}
