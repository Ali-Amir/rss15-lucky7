package orc;

import java.io.*;

public class AX12Servo
{
    Orc orc;
    int id;
    public boolean verbose;
    static final int INST_PING = 1;
    static final int INST_READ_DATA = 2;
    static final int INST_WRITE_DATA = 3;
    static final int INST_REG_WRITE = 4;
    static final int INST_ACTION = 5;
    static final int INST_RESET_DATA = 6;
    static final int INST_SYNC_WRITE = 131;
    static final int ADDR_ID = 3;
    public static final int ERROR_INSTRUCTION = 64;
    public static final int ERROR_OVERLOAD = 32;
    public static final int ERROR_CHECKSUM = 16;
    public static final int ERROR_RANGE = 8;
    public static final int ERROR_OVERHEAT = 4;
    public static final int ERROR_ANGLE_LIMIT = 2;
    public static final int ERROR_VOLTAGE = 1;
    public static final int BROADCAST_ADDRESS = 254;
    double pos0;
    double val0;
    double pos1;
    double val1;
    
    public AX12Servo(final Orc orc, final int id) {
        super();
        this.verbose = false;
        this.pos0 = 0.0;
        this.val0 = 0.0;
        this.pos1 = 300.0;
        this.val1 = 300.0;
        this.orc = orc;
        this.id = id;
    }
    
    public AX12Servo(final Orc orc, final int id, final double pos0, final double val0, final double pos1, final double val1) {
        this(orc, id);
        this.remap(pos0, val0, pos1, val1);
    }
    
    public void remap(final double pos0, final double val0, final double pos1, final double val1) {
        this.pos0 = pos0;
        this.val0 = val0;
        this.pos1 = pos1;
        this.val1 = val1;
    }
    
    double map(final double val) {
        final double x = (val - this.val0) / (this.val1 - this.val0);
        return this.pos0 + x * (this.pos1 - this.pos0);
    }
    
    double unmap(final double pos) {
        final double x = (pos - this.pos0) / (this.pos1 - this.pos0);
        return this.val0 + x * (this.val1 - this.val0);
    }
    
    static byte[] makeCommand(final int id, final int instruction, final byte[] parameters) {
        final int parameterlen = (parameters == null) ? 0 : parameters.length;
        final byte[] cmd = new byte[6 + parameterlen];
        cmd[1] = (cmd[0] = -1);
        cmd[2] = (byte)id;
        cmd[3] = (byte)(parameterlen + 2);
        cmd[4] = (byte)instruction;
        if (parameters != null) {
            for (int i = 0; i < parameters.length; ++i) {
                cmd[5 + i] = parameters[i];
            }
        }
        int checksum = 0;
        for (int j = 2; j < cmd.length - 1; ++j) {
            checksum += (cmd[j] & 0xFF);
        }
        cmd[5 + parameterlen] = (byte)(checksum ^ 0xFF);
        return cmd;
    }
    
    public void changeServoID(final int oldID, final int newID) {
        final OrcResponse resp = this.orc.doCommand(16808448, makeCommand((byte)(oldID & 0xFF), 3, new byte[] { 3, (byte)(newID & 0xFF) }));
        if (this.verbose) {
            resp.print();
        }
    }
    
    public boolean ping() {
        final OrcResponse resp = this.orc.doCommand(16808448, makeCommand(this.id, 1, null));
        if (this.verbose) {
            resp.print();
        }
        final byte[] tmp = { 6, -1, -1, (byte)this.id, 2, 0 };
        for (int i = 0; i < tmp.length; ++i) {
            if (tmp[i] != resp.responseBuffer[resp.responseBufferOffset + i]) {
                return false;
            }
        }
        return true;
    }
    
    public void setGoalDegrees(double deg, final double speedfrac, final double torquefrac) {
        deg = this.map(deg);
        final int posv = (int)(1023.0 * deg / 300.0);
        final int speedv = (int)(1023.0 * speedfrac);
        final int torquev = (int)(1023.0 * torquefrac);
        final OrcResponse resp = this.orc.doCommand(16808448, makeCommand(this.id, 3, new byte[] { 30, (byte)(posv & 0xFF), (byte)(posv >> 8), (byte)(speedv & 0xFF), (byte)(speedv >> 8), (byte)(torquev & 0xFF), (byte)(torquev >> 8) }));
        if (this.verbose) {
            resp.print();
        }
    }
    
    public AX12Status getStatus() {
        final AX12Status status = new AX12Status();
        while (true) {
            final OrcResponse resp = this.orc.doCommand(16808448, makeCommand(this.id, 2, new byte[] { 36, 8 }));
            if (this.verbose) {
                resp.print();
            }
            final DataInputStream ins = resp.ins;
            try {
                final int len = ins.read();
                if (len != 14) {
                    continue;
                }
                int ff = ins.read();
                if (ff != 255) {
                    continue;
                }
                ff = ins.read();
                if (ff != 255) {
                    continue;
                }
                final int id = ins.read();
                if (id != this.id) {
                    continue;
                }
                final int paramlen = ins.read();
                if (paramlen != 10) {
                    continue;
                }
                status.error_flags = ins.read();
                status.positionDegrees = this.unmap(((ins.read() & 0xFF) + ((ins.read() & 0x3F) << 8)) * 300.0 / 1024.0);
                final int ispeed = (ins.read() & 0xFF) + ((ins.read() & 0xFF) << 8);
                status.speed = (ispeed & 0x3FF) / 1024.0;
                if ((ispeed & 0x400) != 0x0) {
                    final AX12Status ax12Status = status;
                    ax12Status.speed *= -1.0;
                }
                final int iload = (ins.read() & 0xFF) + ((ins.read() & 0xFF) << 8);
                status.load = (iload & 0x3FF) / 1024.0;
                if ((iload & 0x400) != 0x0) {
                    final AX12Status ax12Status2 = status;
                    ax12Status2.load *= -1.0;
                }
                status.utimeOrc = resp.utimeOrc;
                status.utimeHost = resp.utimeHost;
                status.voltage = (ins.read() & 0xFF) / 10.0;
                status.temperature = (ins.read() & 0xFF);
            }
            catch (IOException ex) {
                continue;
            }
            break;
        }
        return status;
    }
    
    public int getId() {
        return this.id;
    }
    
    public static void main(final String[] args) {
        final Orc orc = Orc.makeOrc();
        final AX12Servo servo = new AX12Servo(orc, 1);
        int iter = 0;
        while (true) {
            System.out.printf("%d\n", iter);
            servo.ping();
            final AX12Status st = servo.getStatus();
            st.print();
            System.out.println("**");
            servo.setGoalDegrees(150.0, 0.1, 0.1);
            try {
                Thread.sleep(500L);
            }
            catch (InterruptedException ex) {}
            servo.setGoalDegrees(160.0, 0.1, 0.1);
            try {
                Thread.sleep(500L);
            }
            catch (InterruptedException ex2) {}
            ++iter;
        }
    }
}
