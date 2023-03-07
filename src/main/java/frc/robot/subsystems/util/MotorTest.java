package frc.robot.subsystems.util;

import edu.wpi.first.hal.can.CANMessageNotFoundException;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.util.MotorTest;
import frc.robot.subsystems.util.Motor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;
import com.revrobotics.REVLibError;

import SushiFrcLib.Motor.MotorHelper;

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

  private static int numMotors;

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

    // numMotors = motorList.size();
  }

  public void updateMotors() {
    tableArray = dataTable.get();
    if (running.get()) {
      for (int i = 0; i < motorList.size(); i++) {
        if (tableArray.length > i && !tableArray[i].equals("containsNull")) {
          coastOrBrake(i);
          invertMotor(i);
          setCurrentLimit(i);
          setEncoderLimit(i);
          setSpeed(i);
        } else {
          motorList.get(i).disable();
        }
        motorList.get(i).checkElecErrors();
        errorList = motorList.get(i).getErrors(); // array list of strings
        if (errorList != null) {
          errorArray.add(String.join(" ", errorList));
        }
      }

      for (int i = 0; i < solenoidList.size(); i++) {
        setSolenoid(i);
      }

      if (errorArray != null) {
        errorTable.set(errorArray.toArray(new String[motorList.size()]));
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

  public void coastOrBrake(int idx) {
    String[] motorArray = (tableArray[idx]).split(" ");
    if ((motorArray[6]) == "false") {
      motorList.get(idx).setIdle(Motor.IdleMode.BRAKE);
    } else {
      motorList.get(idx).setIdle(Motor.IdleMode.COAST);
    }
  }

  public void invertMotor(int idx) {
    String[] motorArray = (tableArray[idx]).split(" ");
    boolean isInverted = Boolean.parseBoolean(motorArray[7]);
    motorList.get(idx).invertMotor(isInverted);
  }

  public void setSpeed(int idx) {
    if (tableArray[idx] != null) {
      String[] motorArray = (tableArray[idx]).split(" ");
      double constSpeed = (Double.parseDouble(motorArray[4]));
      boolean isJoystick = (Boolean.parseBoolean(motorArray[5])); // make boolean
      motorList.get(idx).setSpeed(constSpeed, isJoystick);
    } else {
      motorList.get(idx).disable();
    }
  }

  public void stopMotors(Motor motor) {
    motor.disable();
  }

  public void setCurrentLimit(int idx) {
    String[] motorArray = (tableArray[idx]).split(" ");
    int currLimit = (int) (Double.parseDouble(motorArray[8]));
    motorList.get(idx).setCurrentLimit(currLimit);
  }

  public void setEncoderLimit(int idx) {
    String[] motorArray = (tableArray[idx]).split(" ");
    double low = (Double.parseDouble(motorArray[9]));
    double high = (Double.parseDouble(motorArray[10]));
    motorList.get(idx).setEncoderLimit(low, high);
  }

  public void setSolenoid(int idx) {
    String[] motorArray = (tableArray[idx]).split(" ");
    Value value;
    if (Boolean.parseBoolean(motorArray[11])) {
      value = Value.kForward;
    } else {
      value = Value.kReverse;
    }
    solenoidList.get(idx).set(value);
  }

  public void register(Motor motor, DoubleSolenoid solenoid, String subsystem, String name, int id, double currentLimit,
      int pdhPort) {
    if (solenoid == null) {
      motorList.add(motor);
    } else {
      solenoidList.add(solenoid);
    }

    motorArray.add(subsystem + " " + name + " " + id + " " + pdhPort + " " + currentLimit + " 0 0 0 0.0 -2.0 0.0 0 0");
    motorTable.set(motorArray.toArray(new String[motorList.size() + solenoidList.size()]));
    numMotors = motorList.size() + solenoidList.size();
  }

}
