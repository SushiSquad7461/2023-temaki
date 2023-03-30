import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Consumer;

public class CustomToggleButton {
    private String name;
    private ToggleButton button;

    public CustomToggleButton(String name, Consumer<Boolean> callback) { //consumer takes in data
        this.name = name;
        button = new ToggleButton();

        SmartDashboard.putBoolean(name, false); //places boolean to associate toggle with, default false
        SmartDashboard.putData(name, button);

        button.addToggleButtonListener((source, value) -> {
            callback.accept(value); //set consumer value, value is newly changed state
        });
    }
}
