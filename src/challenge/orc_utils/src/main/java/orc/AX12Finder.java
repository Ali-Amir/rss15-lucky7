package orc;

import java.util.*;

public class AX12Finder
{
    public static void main(final String[] args) {
        final Orc orc = Orc.makeOrc();
        final ArrayList<Integer> servoIDs = new ArrayList<Integer>();
        System.out.println("Attempting to find servos: ");
        for (int i = 1; i <= 253; ++i) {
            final AX12Servo servo = new AX12Servo(orc, i);
            final boolean result = servo.ping();
            if (result) {
                servoIDs.add(i);
                System.out.print("o");
            }
            else {
                System.out.print("x");
            }
            if (i % 64 == 0) {
                System.out.println();
            }
        }
        System.out.print("\nFound servos: [");
        for (final Integer j : servoIDs) {
            System.out.print(j + ",");
        }
        System.out.println("]");
    }
}
