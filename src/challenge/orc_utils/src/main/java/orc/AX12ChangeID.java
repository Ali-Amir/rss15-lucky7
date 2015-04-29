package orc;

public class AX12ChangeID
{
    public static void main(final String[] args) {
        int oldID = 1;
        int newID = 2;
        final Orc orc = Orc.makeOrc();
        if (args != null) {
            System.out.println("length = " + args.length);
            if (args.length == 1) {
                final Integer i = new Integer(args[0]);
                newID = i;
            }
            else if (args.length == 2) {
                System.out.println(args[0]);
                Integer i = new Integer(args[0]);
                oldID = i;
                i = new Integer(args[1]);
                newID = i;
            }
        }
        System.out.println("oldID=" + oldID + "\t\tnewID=" + newID);
        final AX12Servo servo = new AX12Servo(orc, oldID);
        if (servo.ping()) {
            System.out.println("GOOD Ping");
            servo.changeServoID(oldID, newID);
        }
        else {
            System.out.println("BAD Ping");
        }
    }
}
