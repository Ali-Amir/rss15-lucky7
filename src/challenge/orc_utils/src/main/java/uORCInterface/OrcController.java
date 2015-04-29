package uORCInterface;

import orc.*;

public class OrcController implements OrcControlInterface
{
    private Orc orc;
    private Motor[] motorSet;
    private int[] portSet;
    protected static final int NUM_DIGITAL_IO = 16;
    private static final boolean[] DIGITAL_SETUP;
    private DigitalInput[] dIn;
    private DigitalOutput[] dOut;
    protected static final int MAX_ANALOG_PORT = 7;
    protected static final int MAX_PWM = 255;
    
    public OrcController(final int[] array) {
        super();
        this.dIn = new DigitalInput[16];
        this.dOut = new DigitalOutput[16];
        this.orc = Orc.makeOrc();
        if (array.length > 2) {
            System.out.println("Too many Motors");
            System.exit(1);
        }
        this.motorSet = new Motor[array.length];
        this.portSet = new int[array.length];
        for (int i = 0; i < array.length; ++i) {
            this.checkPortRange(array[i]);
            this.motorSet[i] = new Motor(this.orc, array[i], false);
            this.portSet[i] = array[i];
        }
        for (int j = 0; j < 16; ++j) {
            if (OrcController.DIGITAL_SETUP[j]) {
                this.dIn[j] = new DigitalInput(this.orc, j, true, false);
            }
            else {
                this.dOut[j] = new DigitalOutput(this.orc, j);
            }
        }
        System.out.println("uOrcBoard initialized...");
    }
    
    public double analogRead(final int n) {
        if (n < 0 || n > 7) {
            System.out.println("Out of bound: the analog port must be 0~7");
            System.exit(1);
        }
        return new AnalogInput(this.orc, n).getVoltage();
    }
    
    public long clockReadSlave() {
        return this.orc.getStatus().utimeOrc;
    }
    
    public void motorSet(final int n, final int a) {
        this.checkMotorRange(n);
        double pwm;
        if (a < -255 || a > 255) {
            System.out.println("WARNING Out of bound: pwm should be [-255 255]");
            pwm = a / Math.abs(a);
        }
        else {
            pwm = a / 255.0;
        }
        this.motorSet[n].setPWM(pwm);
    }
    
    public void motorSlewWrite(final int n, final int n2) {
        this.checkMotorRange(n);
        this.motorSet[n].setSlewSeconds(n2);
    }
    
    public int readEncoder(final int n) {
        this.checkMotorRange(n);
        return new QuadratureEncoder(this.orc, this.portSet[n], false).getPosition();
    }
    
    public int readVelocity(final int n) {
        this.checkMotorRange(n);
        return (int)new QuadratureEncoder(this.orc, this.portSet[n], false).getVelocity();
    }
    
    public void servoWrite(final int n, final int n2) {
    }
    
    public void digitalSet(final int n, final boolean value) {
        if (OrcController.DIGITAL_SETUP[n]) {
            System.out.println("This port is NOT set for digital OUTPUT!!!");
            System.exit(1);
        }
        else {
            this.dOut[n].setValue(value);
        }
    }
    
    public boolean digitalRead(final int n) {
        if (!OrcController.DIGITAL_SETUP[n]) {
            System.out.println("This port is NOT set for digital INPUT!!!");
            System.exit(1);
        }
        return this.dIn[n].getValue();
    }
    
    private void checkMotorRange(final int n) {
        if (n < 0 || n > this.motorSet.length) {
            System.out.println("Out of bound: check the motor number");
            System.exit(1);
        }
    }
    
    private void checkPortRange(final int n) {
        if (n < 0 || n > 2) {
            System.out.println("Out of bound: the port must be 0~2");
            System.exit(1);
        }
    }

    public boolean isSim() {
        return false;
    }

    public void setNode(Object obj) {}
    
    static {
        DIGITAL_SETUP = new boolean[] { true, true, true, true, true, true, true, true, false, false, false, false, false, true, false, true };
    }
}
