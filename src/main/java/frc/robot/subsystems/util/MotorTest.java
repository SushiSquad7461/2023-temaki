package frc.robot.subsystems.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

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
  private Map<String, List<Motor>> motorsMap;
  
  private ArrayList<String> unwantedMotors;

  public static MotorTest getInstance() {
    if (instance == null) {
      instance = new MotorTest();
    }
    return instance;
  }

  private MotorTest() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("dataTable");

    dataTable = table.getStringArrayTopic("tableValues").subscribe(null);
    running = table.getBooleanTopic("Running?").subscribe(false);
    twitchTest = table.getBooleanTopic("twitchTest?").subscribe(false);

    tableArray = dataTable.get();

    motorTable = table.getStringArrayTopic("motors").publish();
    motorArray = new ArrayList<String>();

    errorHandler = ErrorHandler.getInstance();
    motorList = new ArrayList<Motor>();
    solenoidList = new ArrayList<DoubleSolenoid>();

    motorsMap = new HashMap<String, List<Motor>>();
    unwantedMotors = new ArrayList<String>();
    unwantedMotors.add("rightArm");
  }

  public Command runSubsystemTwitch(String subsystem, double waitTime, double speed) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          for (int i = 0; i < (motorsMap.get(subsystem)).size(); i++) {
            Motor motor = motorsMap.get(subsystem).get(i);
            if (unwantedMotors.contains(motor.getName())) {
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
    Double[] initialSpeeds = new Double[swerveStatesBefore.length];
        for( int i=0; i< swerveStatesBefore.length; i++){
          initialSpeeds[i] = swerveStatesBefore[i].speedMetersPerSecond;
        }
    SequentialCommandGroup swerveGroup = new SequentialCommandGroup(
      new InstantCommand(() -> {
        swerve.drive(new Translation2d(), 1, false, true);
      }),
      new WaitCommand(.09),
      
      new InstantCommand(() -> {
        SwerveModuleState[] swerveStates = swerve.getStates();
        for(SwerveModuleState state: swerveStates){
          for (Double initialSpeed: initialSpeeds){
            if (state.speedMetersPerSecond <= initialSpeed) {
              errorHandler.add(state.toString() + " not working");
            }
          }

        }
      }));
        swerve.drive(new Translation2d(), 0, false, true);
        swerveGroup.addRequirements(swerve);
        errorHandler.sendAllErrors();
        return swerveGroup;    
    };


  public Command runTwitch() {
    return new ParallelCommandGroup(
        runSubsystemTwitch("intake", 0.1, 0.5),
        runSubsystemTwitch("manipulator", 0.1, 0.5),
        runSubsystemTwitch("indexer", 0.1, 0.2),
        runSubsystemTwitch("arm", 0.1, 0.01));
  }

  public void runTwitchTest() {
    if (twitchTest.get()) {
      runTwitch().schedule();;
    }
  }

  public void updateMotors() {
    tableArray = dataTable.get();
    if (running.get()) {
      int numMotors = 0;
      int numSolenoid = 0;
      for (int i = 0; i < tableArray.length; i++) {
        String[] deviceArray = (tableArray[i]).split(" ");
        if (tableArray.length > i) {
          if (!tableArray[i].contains("solenoid")) {
            if (deviceArray[deviceArray.length - 1].equals("true")) {
              coastOrBrake(deviceArray, numMotors);
              invertMotor(deviceArray, numMotors);
              setCurrentLimit(deviceArray, numMotors);
              setEncoderLimit(deviceArray, numMotors);
              setSpeed(deviceArray, numMotors);
              motorList.get(numMotors).checkElecErrors();
            } else {
              motorList.get(numMotors).disable();
            }
            numMotors++;
          } else {
            if (deviceArray[deviceArray.length - 1].equals("true")) {
              setSolenoid(deviceArray, numSolenoid);
            }
            numSolenoid++;
          }
        }
      }
      errorHandler.sendAllErrors();
    } else {
      isStop(tableArray);
    }

  }

  public void isStop(String[] tableArray) {
    for (int i = 0; i < motorList.size(); i++) {
      stopMotors(motorList.get(i));
    }
  }

  public void coastOrBrake(String[] deviceArray, int numMotors) {
    if ((deviceArray[DiagnosticConstants.COAST_IDX]) == "false") {
      motorList.get(numMotors).setIdle(Motor.IdleMode.BRAKE);
    } else {
      motorList.get(numMotors).setIdle(Motor.IdleMode.COAST);
    }
  }

  public void invertMotor(String[] deviceArray, int numMotors) {
    boolean isInverted = Boolean.parseBoolean(deviceArray[DiagnosticConstants.INVERT_IDX]);
    motorList.get(numMotors).invertMotor(isInverted);
  }

  public void setSpeed(String[] deviceArray, int numMotors) {
    double constSpeed = (Double.parseDouble(deviceArray[DiagnosticConstants.CONSTANT_SPEED_IDX]));
    boolean isJoystick = (Boolean.parseBoolean(deviceArray[DiagnosticConstants.JOYSTICK_IDX]));
    motorList.get(numMotors).setSpeed(constSpeed, isJoystick);
  }

  public void stopMotors(Motor motor) {
    motor.disable();
  }

  public void setCurrentLimit(String[] deviceArray, int numMotors) {
    int currLimit = (int) (Double.parseDouble(deviceArray[DiagnosticConstants.CURR_LIMIT_IDX]));
    motorList.get(numMotors).setCurrentLimit(currLimit);
  }

  public void setEncoderLimit(String[] deviceArray, int numMotors) {
    double low = (Double.parseDouble(deviceArray[DiagnosticConstants.ENCODER_LOW_IDX]));
    double high = (Double.parseDouble(deviceArray[DiagnosticConstants.ENCODER_HIGH_IDX]));
    motorList.get(numMotors).setEncoderLimit(low, high);
  }

  public void setSolenoid(String[] deviceArray, int numSolenoid) {
    Value value;
    if (Boolean.parseBoolean(deviceArray[DiagnosticConstants.SWITCH_SOLENOID_IDX])) {
      value = Value.kForward;
    } else {
      value = Value.kReverse;
    }
    solenoidList.get(numSolenoid).set(value);
  }
  
  public void registerSolenoid(DoubleSolenoid solenoid, String subsystem, String name) {
    motorArray.add(subsystem + " " + name + " 0 0 0 0 0 0 0.0 -2.0 0.0 0 0");
    solenoidList.add(solenoid);
    motorTable.set(motorArray.toArray(new String[motorList.size() + solenoidList.size()]));
  }

  public void registerMotor(CANSparkMax motor, String subsystem, String name) {
    registerMotor(new Neo(motor), subsystem, name);
  }

  public void registerMotor(WPI_TalonFX motor, String subsystem, String name) {
    registerMotor(new Falcon(motor), subsystem, name);
  }

  public void registerMotor(Motor motor, String subsystem, String name) {
    motorArray.add(motor.getRegisterString(subsystem, name));
    motorList.add(motor);
    motor.setSubsystem(subsystem);
    motor.setName(name);
    motorTable.set(motorArray.toArray(new String[motorList.size() + solenoidList.size()]));

    if (!motorsMap.containsKey(subsystem)) {
      motorsMap.put(subsystem, new ArrayList<Motor>());
    }

    motorsMap.get(subsystem).add(motor);
  }

  public void unRegisterAlllMotors(){
    motorList.removeAll(motorList);
    motorsMap.clear();
  }
}
