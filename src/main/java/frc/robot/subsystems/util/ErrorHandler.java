package frc.robot.subsystems.util;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;

public class ErrorHandler {
    private NetworkTableInstance inst;
    private NetworkTable table;
  
    private ArrayList<String> allErrors;
    static ErrorHandler instance;
    private StringArrayPublisher errorTable;

    public ErrorHandler() {
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("dataTable");
    
        instance = null;
        errorTable = table.getStringArrayTopic("errors").publish();
    }

    public static ErrorHandler getInstance() {
        if (instance == null) {
          instance = new ErrorHandler();
        }
        return instance;
    }

    public void add(String error) {
        allErrors.add(error);
    }

    public ArrayList<String> getAllErrors(){
        return allErrors;
    }

    public void sendAllErrors() {
        if (getAllErrors() != null) {
            ArrayList<String> errors = getAllErrors();
            errorTable.set(errors.toArray(new String[errors.size()]));
            allErrors.removeAll(allErrors);
        }
    }

}
