package org.deceivers.util;

public class JoystickHelper {
    public double value;

    public JoystickHelper(double input){
        value = input;
    }

    public JoystickHelper applyDeadband(double deadband){
        if (Math.abs(value) < Math.abs(deadband)){
            value = 0;
        }

        return this;
    }

    public JoystickHelper applyPower(double power){
        value = Math.abs(Math.pow(value, power))*Math.signum(value);

        return this;
    }

    public JoystickHelper setInput(double input){
        value = input;
        return this;
    }

}
