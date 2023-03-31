package frc.robot.util;

import java.util.function.Consumer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class NormalButton {

    private GenericEntry button;
    private String name;


    public NormalButton(String name, Consumer<Boolean> callback) { //what does callback do,, listen to button change
        this.name = name;

        button = Shuffleboard.getTab("<some tab>")
            .add(name, false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
    }


    public boolean checkButton() {
        return button.getBoolean(false);
    }

}
