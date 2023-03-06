package frc.robot.subsystems.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class MotorTest {
  private NetworkTableInstance inst;
  private NetworkTable table;

  private String[] tableArray;
  private ArrayList<String> motorArray;
  private ArrayList<String> errorArray;
  private ArrayList<String> errorList;

  private StringArraySubscriber dataTable;
  private BooleanSubscriber running;

  private StringArrayPublisher motorTable;
  private StringArrayPublisher errorTable;

  static MotorTest instance;
  private List<Motor> motorList;
  private List<DoubleSolenoid> solenoidList;

  private int numMotors;
  private int numSolenoid;

  public static MotorTest getInstance() {
    if (instance == null) {
      instance = new MotorTest();
    }
    return instance;
  }

  public MotorTest() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("dataTable");
    dataTable = table.getStringArrayTopic("tableValues").subscribe(null);
    running = table.getBooleanTopic("Running?").subscribe(false);
    tableArray = dataTable.get();

    motorTable = table.getStringArrayTopic("motors").publish();
    motorArray = new ArrayList<String>();

    errorTable = table.getStringArrayTopic("errors").publish();
    errorList = new ArrayList<String>();
    errorArray = new ArrayList<String>();

    instance = null;
    motorList = new ArrayList<Motor>();
    solenoidList = new ArrayList<DoubleSolenoid>();

    numMotors = 0;
    numSolenoid = 0;
  }

  public void updateMotors() {
    tableArray = dataTable.get();
    if (running.get()) {
      numMotors = 0;
      numSolenoid = 0;      
      for (int i = 0; i < tableArray.length; i++) {
        String[] deviceArray = (tableArray[i]).split(" ");
        if (tableArray.length > i) {
          if(!tableArray[i].contains("solenoid")){
            if (deviceArray[deviceArray.length-1].equals("true")){
              coastOrBrake(deviceArray);
              invertMotor(deviceArray);
              setCurrentLimit(deviceArray);
              setEncoderLimit(deviceArray);
              setSpeed(deviceArray);
              motorList.get(numMotors).checkElecErrors();
              errorList = motorList.get(numMotors).getErrors();        
              if (errorList != null) {
                errorArray.add(String.join(" ", errorList));
              }
            }
            else {
              motorList.get(numMotors).disable();
            }
            numMotors++;
          }
          else{
            if(deviceArray[deviceArray.length-1].equals("true")){
              setSolenoid(deviceArray);
            }
            numSolenoid++;
          }
          
        }
      }

      if (errorArray != null) {
        errorTable.set(errorArray.toArray(new String[motorList.size()]));
        errorArray.removeAll(errorArray);
      }
    } else {
      isStop(tableArray);
    }

  }

  public void isStop(String[] tableArray) {
    for (int i = 0; i < motorList.size(); i++) {
      stopMotors(motorList.get(i));
    }
  }

  public void coastOrBrake(String[] deviceArray) { 
    if ((deviceArray[6]) == "false") {
      motorList.get(numMotors).setIdle(Motor.IdleMode.BRAKE);
    } else {
      motorList.get(numMotors).setIdle(Motor.IdleMode.COAST);
    }
  }

  public void invertMotor(String[] deviceArray) {
    boolean isInverted = Boolean.parseBoolean(deviceArray[7]);
    motorList.get(numMotors).invertMotor(isInverted);
  }

  public void setSpeed(String[] deviceArray) {
    double constSpeed = (Double.parseDouble(deviceArray[4]));
    boolean isJoystick = (Boolean.parseBoolean(deviceArray[5])); // make boolean
    motorList.get(numMotors).setSpeed(constSpeed, isJoystick);
  }

  public void stopMotors(Motor motor) {
    motor.disable();
  }

  public void setCurrentLimit(String[] deviceArray) {
    int currLimit = (int) (Double.parseDouble(deviceArray[8]));
    motorList.get(numMotors).setCurrentLimit(currLimit);
  }

  public void setEncoderLimit(String[] deviceArray) {
    double low = (Double.parseDouble(deviceArray[9]));
    double high = (Double.parseDouble(deviceArray[10]));
    motorList.get(numMotors).setEncoderLimit(low, high);
  }

  public void setSolenoid(String[] deviceArray) {
    Value value;
    if (Boolean.parseBoolean(deviceArray[12])) {
      value = Value.kForward;
    } else {
      value = Value.kReverse;
    }
    solenoidList.get(numSolenoid).set(value);
  }

  public void register(Motor motor, DoubleSolenoid solenoid, String subsystem, String name) {

    if (solenoid == null) {
      motorArray.add(motor.getRegisterString(subsystem, name));
      motorList.add(motor);
    } else {
      motorArray.add(subsystem + " " + name +  " 0 0 0 0 0 0 0.0 -2.0 0.0 0 0");
      solenoidList.add(solenoid);
    }

    motorTable.set(motorArray.toArray(new String[motorList.size() + solenoidList.size()]));
  }

}
