package frc.robot.util;

import edu.wpi.first.wpilibj.PWM;

public class LED {
    PWM led;
    public LED(int port) {
        led = new PWM(port);
    }

    public void set(double value, double pulseWidth) {
        pulseWidth/=1000.0;
        led.setBounds(2.005, value+0.005, value, value-0.005, 0.995);
        led.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
        led.setSpeed(value);
        led.setZeroLatch();
    }
}
