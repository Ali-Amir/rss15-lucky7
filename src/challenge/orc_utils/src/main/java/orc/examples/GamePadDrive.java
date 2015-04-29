package orc.examples;

import orc.util.*;
import orc.*;

public class GamePadDrive
{
    public static void main(final String[] args) {
        final GamePad gp = new GamePad();
        final Orc orc = Orc.makeOrc();
        final boolean flipLeft = args.length > 2 && Boolean.parseBoolean(args[1]);
        final boolean flipRight = args.length <= 2 || Boolean.parseBoolean(args[2]);
        final boolean flipMotors = args.length > 3 && Boolean.parseBoolean(args[3]);
        final Motor leftMotor = new Motor(orc, flipMotors ? 1 : 0, flipLeft);
        final Motor rightMotor = new Motor(orc, flipMotors ? 0 : 1, flipRight);
        System.out.println("flipLeft: " + flipLeft + ", flipRight: " + flipRight + ", flipMotors: " + flipMotors);
        System.out.println("Hit any gamepad button to begin.");
        gp.waitForAnyButtonPress();
        System.out.printf("%15s %15s %15s %15s\n", "left", "right", "left current", "right current");
        while (true) {
            double left = 0.0;
            double right = 0.0;
            final double fwd = -gp.getAxis(3);
            final double lr = gp.getAxis(2);
            left = fwd - lr;
            right = fwd + lr;
            final double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }
            System.out.printf("%15f %15f %15f %15f\r", left, right, leftMotor.getCurrent(), rightMotor.getCurrent());
            leftMotor.setPWM(left);
            rightMotor.setPWM(right);
            try {
                Thread.sleep(30L);
            }
            catch (InterruptedException ex) {}
        }
    }
}
