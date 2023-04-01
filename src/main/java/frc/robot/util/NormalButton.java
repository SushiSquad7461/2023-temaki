package frc.robot.util;

import java.util.function.Consumer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class NormalButton {

    private GenericEntry button;
    private String name;
    private Consumer<Boolean> callback;

    public NormalButton(String name, Consumer<Boolean> callback) {
        this.name = name;
        this.callback = callback;

        button = Shuffleboard.getTab("<some tab>")
            .add(name, false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .getEntry();
    }

    public void checkButton() { //call in periodic to actually check value, no listeners for changes built in
        if(button.getBoolean(false)){
            callback.accept(button.getBoolean(false));
        }
    }

}
