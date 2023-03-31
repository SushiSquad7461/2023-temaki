package frc.robot.util;

import java.util.function.Consumer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ToggleButton {

    private GenericEntry button;
    private String name;


    public ToggleButton(String name, Consumer<Boolean> callback) {
        this.name = name;

        button = Shuffleboard.getTab("<some tab>")
            .add(name, false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
    }


    public boolean getButton() {
        return button.getBoolean(false);
    }

}

