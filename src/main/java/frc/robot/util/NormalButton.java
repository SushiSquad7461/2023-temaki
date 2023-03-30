import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Consumer;

public class NormalButton {
    private String name;
    private Button button;

    public NormalButton(String name, Consumer<Boolean> callback) {
        this.name = name;
        button = new Button();

        SmartDashboard.putData(name, button);

        button.whenPressed(() -> {
            callback.accept(true);
        });
    }
}
