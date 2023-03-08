package frc.robot.subsystems.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;

public class MotorTest {
  private NetworkTableInstance inst;
  private NetworkTable table;

  private String[] tableArray;
  private ArrayList<String> motorArray;

  private StringArraySubscriber dataTable;
  private BooleanSubscriber running;
  private BooleanSubscriber twitchTest;

  private StringArrayPublisher motorTable;

  private ErrorHandler errorHandler;

  static MotorTest instance;
  private List<Motor> motorList;
  private List<DoubleSolenoid> solenoidList;

  private int numMotors;
  private int numSolenoid;

  private Map<String, List<Motor>> motorsMap;
  private CommandBase commandBase;

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
    twitchTest = table.getBooleanTopic("twitchTest?").subscribe(false);

    tableArray = dataTable.get();

    motorTable = table.getStringArrayTopic("motors").publish();
    motorArray = new ArrayList<String>();

    errorHandler = ErrorHandler.getInstance();

    instance = null;
    motorList = new ArrayList<Motor>();
    solenoidList = new ArrayList<DoubleSolenoid>();

    numMotors = 0;
    numSolenoid = 0;

    motorsMap = new HashMap<String, List<Motor>>();
    commandBase = new CommandBase() {
    };
  }

  public Command runSubsystemTwitch(String subsystem, double waitTime, double speed) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          for (int i = 0; i < (motorsMap.get(subsystem)).size(); i++) {
            Motor motor = motorsMap.get(subsystem).get(i);
            if (motor.getName().equals("rightArm")) {
              continue;
            }
            motor.startTwitch(speed);
          }
        }),
        new WaitCommand(waitTime),
        new InstantCommand(() -> {
          for (int i = 0; i < (motorsMap.get(subsystem)).size(); i++) {
            Motor motor = motorsMap.get(subsystem).get(i);
            motor.endTwitch();
            motor.checkEncoderErrors();
          }
          errorHandler.sendAllErrors();
        }));
  }

  public Command runSwerveTwitch() {
    Swerve swerve = Swerve.getInstance();
    SwerveModuleState[] swerveStatesBefore = swerve.getStates();
    Rotation2d[] initialRotations = new Rotation2d[swerveStatesBefore.length];
        for( int i=0; i< swerveStatesBefore.length; i++){
          initialRotations[i] = swerveStatesBefore[i].angle;
        }
    SequentialCommandGroup swerveGroup = new SequentialCommandGroup(
      new InstantCommand(() -> {
        swerve.drive(new Translation2d(), 1, false, true);
      }),
      new WaitCommand(.1),
      
      new InstantCommand(() -> {
        SwerveModuleState[] swerveStates = swerve.getStates();
        for( SwerveModuleState state: swerveStates){
          if(state.angle > initial){

          }
        }
        swerve.drive(new Translation2d(), 0, false, true);
      }));
    swerveGroup.addRequirements(swerve);
    return swerveGroup;
  }

  public Command runTwitch() {
    return new ParallelCommandGroup(
        runSubsystemTwitch("intake", 0.1, 0.5),
        runSubsystemTwitch("manipulator", 0.1, 0.5),
        runSubsystemTwitch("indexer", 0.1, 0.2),
        runSubsystemTwitch("arm", 0.1, 0.01));
  }

  public void runTwitchTest() {
    if (twitchTest.get()) {
      runTwitch();
    }
  }

  public void updateMotors() {
    tableArray = dataTable.get();
    if (running.get()) {
      numMotors = 0;
      numSolenoid = 0;
      for (int i = 0; i < tableArray.length; i++) {
        String[] deviceArray = (tableArray[i]).split(" ");
        if (tableArray.length > i) {
          if (!tableArray[i].contains("solenoid")) {
            if (deviceArray[deviceArray.length - 1].equals("true")) {
              coastOrBrake(deviceArray);
              invertMotor(deviceArray);
              setCurrentLimit(deviceArray);
              setEncoderLimit(deviceArray);
              setSpeed(deviceArray);
              motorList.get(numMotors).checkElecErrors();
            } else {
              motorList.get(numMotors).disable();
            }
            numMotors++;
          } else {
            if (deviceArray[deviceArray.length - 1].equals("true")) {
              setSolenoid(deviceArray);
            }
            numSolenoid++;
          }
        }
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
      motor.setSubsystem(subsystem);
      motor.setName(name);
    } else {
      motorArray.add(subsystem + " " + name + " 0 0 0 0 0 0 0.0 -2.0 0.0 0 0");
      solenoidList.add(solenoid);
    }

    motorTable.set(motorArray.toArray(new String[motorList.size() + solenoidList.size()]));
    if (!motorsMap.containsKey(subsystem)) {
      motorsMap.put(subsystem, new ArrayList<Motor>());
    }

    ((ArrayList<Motor>) motorsMap.get(subsystem)).add(motor);
  }

}
