package frc.robot.subsystems.util;

import java.util.ArrayList;

import edu.wpi.first.networktables.StringArrayPublisher;

public class ErrorHandler {
    private ArrayList<String> allErrors;
    static ErrorHandler instance;

    public ErrorHandler() {
        instance = null;
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

    public void clear(){
        allErrors.removeAll(allErrors);
    }

}
