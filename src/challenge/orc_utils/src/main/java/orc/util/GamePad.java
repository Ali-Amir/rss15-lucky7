package orc.util;

import java.io.*;

public class GamePad
{
    String devicePath;
    int[] axes;
    int[] buttons;
    
    public GamePad() {
        super();
        this.axes = new int[16];
        this.buttons = new int[16];
        final String[] paths = { "/dev/js0", "/dev/input/js0" };
        for (int i = 0; i < paths.length; ++i) {
            final String path = paths[i];
            final File f = new File(path);
            if (f.exists()) {
                this.devicePath = path;
                break;
            }
        }
        if (this.devicePath == null) {
            System.out.println("Couldn't find a joystick.");
            System.exit(-1);
        }
        new ReaderThread().start();
    }
    
    public GamePad(final String path) {
        super();
        this.axes = new int[16];
        this.buttons = new int[16];
        this.devicePath = path;
        new ReaderThread().start();
    }
    
    public double getAxis(final int axis) {
        if (axis >= this.axes.length) {
            return 0.0;
        }
        return this.axes[axis] / 32767.0;
    }
    
    public boolean getButton(final int button) {
        return button < this.buttons.length && this.buttons[button] > 0;
    }
    
    public int waitForAnyButtonPress() {
        final boolean[] buttonState = new boolean[16];
        for (int i = 0; i < buttonState.length; ++i) {
            buttonState[i] = this.getButton(i);
        }
        int i = 0;
    Block_3:
        while (true) {
            for (i = 0; i < buttonState.length; ++i) {
                if (this.getButton(i) != buttonState[i]) {
                    break Block_3;
                }
            }
            try {
                Thread.sleep(10L);
            }
            catch (InterruptedException ex) {}
        }
        return i;
    }
    
    public static void main(final String[] args) {
        final GamePad gp = new GamePad();
        final int nAxes = 6;
        final int nButtons = 16;
        for (int i = 0; i < nAxes; ++i) {
            System.out.printf("%10s ", "Axis " + i);
        }
        for (int i = 0; i < nButtons; ++i) {
            final int v = i & 0xF;
            System.out.printf("%c", (v >= 10) ? (97 + (v - 10)) : (48 + v));
        }
        System.out.printf("\n", new Object[0]);
        while (true) {
            String s = "";
            for (int j = 0; j < nButtons; ++j) {
                s += (gp.getButton(j) ? 1 : 0);
            }
            System.out.printf("\r", new Object[0]);
            for (int j = 0; j < nAxes; ++j) {
                System.out.printf("%10f ", gp.getAxis(j));
            }
            System.out.printf("%s", s);
            try {
                Thread.sleep(10L);
            }
            catch (InterruptedException ex) {}
        }
    }
    
    class ReaderThread extends Thread
    {
        ReaderThread() {
            super();
            this.setDaemon(true);
        }
        
        public void run() {
            try {
                this.runEx();
            }
            catch (IOException ex) {
                System.out.println("GamePad ex: " + ex);
            }
        }
        
        public void runEx() throws IOException {
            final FileInputStream fins = new FileInputStream(new File(GamePad.this.devicePath));
            final byte[] buf = new byte[8];
            while (true) {
                fins.read(buf);
                final int mstime = (buf[0] & 0xFF) | (buf[1] & 0xFF) << 8 | (buf[2] & 0xFF) << 16 | (buf[3] & 0xFF) << 24;
                int value = (buf[4] & 0xFF) | (buf[5] & 0xFF) << 8;
                if ((value & 0x8000) > 0) {
                    value |= 0xFFFF0000;
                }
                final int type = buf[6] & 0xFF;
                final int number = buf[7] & 0xFF;
                if ((type & 0x3) == 0x1) {
                    if (number < GamePad.this.buttons.length) {
                        GamePad.this.buttons[number] = value;
                    }
                    else {
                        System.out.println("GamePad: " + number + " buttons!");
                    }
                }
                if ((type & 0x3) == 0x2) {
                    if (number < GamePad.this.axes.length) {
                        GamePad.this.axes[number] = value;
                    }
                    else {
                        System.out.println("GamePad: " + number + " axes!");
                    }
                }
            }
        }
    }
}
