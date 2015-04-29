package orc.spy;

import javax.swing.*;
import orc.*;
import java.awt.event.*;
import java.awt.*;

public class Spy
{
    JFrame jf;
    JDesktopPane jdp;
    TextPanelWidget basicDisplay;
    TextPanelWidget analogDisplay;
    TextPanelWidget qeiDisplay;
    TextPanelWidget digDisplay;
    MotorPanel[] motorPanels;
    ServoPanel[] servoPanels;
    Orc orc;
    int xpos;
    int ypos;
    boolean firstStatus;
    
    public static void main(final String[] args) {
        new Spy(args);
    }
    
    public Spy(final String[] args) {
        super();
        this.jdp = new JDesktopPane();
        this.xpos = 0;
        this.ypos = 0;
        this.firstStatus = true;
        if (args.length > 0) {
            this.orc = Orc.makeOrc(args[0]);
        }
        else {
            this.orc = Orc.makeOrc();
        }
        (this.jf = new JFrame("OrcSpy: " + this.orc.getAddress())).setLayout(new BorderLayout());
        this.jf.add(this.jdp, "Center");
        this.basicDisplay = new TextPanelWidget(new String[] { "Parameter", "Value" }, new String[][] { { "uorc time", "0" } }, new double[] { 0.4, 0.6 });
        this.analogDisplay = new TextPanelWidget(new String[] { "Port", "Raw value", "Value", "LPF Value" }, new String[][] { { "0", "0", "0", "0" }, { "1", "0", "0", "0" }, { "2", "0", "0", "0" }, { "3", "0", "0", "0" }, { "4", "0", "0", "0" }, { "5", "0", "0", "0" }, { "6", "0", "0", "0" }, { "7", "0", "0", "0" }, { " 8 (mot0)", "0", "0", "0" }, { " 9 (mot1)", "0", "0", "0" }, { "10 (mot2)", "0", "0", "0" }, { "11 (vbat)", "0", "0", "0" }, { "12 (temp)", "0", "0", "0" } }, new double[] { 0.3, 0.5, 0.5, 0.5 });
        this.digDisplay = new TextPanelWidget(new String[] { "Port", "Mode", "Value" }, new String[][] { { "0", "--", "0" }, { "1", "--", "0" }, { "2", "--", "0" }, { "3", "--", "0" }, { "4", "--", "0" }, { "5", "--", "0" }, { "6", "--", "0" }, { "7", "--", "0" }, { "8  (mot0 fail)", "--", "0" }, { "9  (mot0 en)  ", "--", "0" }, { "10 (mot1 fail)", "--", "0" }, { "11 (mot1 en)  ", "--", "0" }, { "12 (mot2 fail)", "--", "0" }, { "13 (mot2 en)  ", "--", "0" }, { "14 (estop)    ", "--", "0" }, { "15 (button0)  ", "--", "0" } }, new double[] { 0.3, 0.5, 0.5 });
        this.qeiDisplay = new TextPanelWidget(new String[] { "Port", "Position", "Velocity" }, new String[][] { { "0", "0", "0" }, { "1", "0", "0" } }, new double[] { 0.3, 0.5, 0.5 });
        final JPanel motDisplay = new JPanel();
        final WeightedGridLayout motorWGL = new WeightedGridLayout(new double[] { 0.0, 0.0, 0.8, 0.2 });
        motorWGL.setDefaultRowWeight(1.0);
        motorWGL.setRowWeight(0, 0.0);
        motDisplay.setLayout(motorWGL);
        motDisplay.add(new JLabel("# "));
        motDisplay.add(new JLabel("En "));
        motDisplay.add(new JLabel("PWM "));
        motDisplay.add(new JLabel("Slew"));
        this.motorPanels = new MotorPanel[3];
        for (int i = 0; i < this.motorPanels.length; ++i) {
            this.motorPanels[i] = new MotorPanel(motDisplay, i);
        }
        final JPanel servoDisplay = new JPanel();
        servoDisplay.setLayout(new GridLayout(8, 1));
        this.servoPanels = new ServoPanel[8];
        for (int j = 0; j < this.servoPanels.length; ++j) {
            this.servoPanels[j] = new ServoPanel(j);
            final JPanel jp = new JPanel();
            jp.setLayout(new BorderLayout());
            jp.add(new JLabel(" " + j), "West");
            jp.add(this.servoPanels[j], "Center");
            servoDisplay.add(jp);
        }
        this.jf.setSize(906, 550);
        this.jf.setVisible(true);
        this.makeInternalFrame("Basic Properties", this.basicDisplay, 300, 120);
        this.makeInternalFrame("Analog Input", this.analogDisplay, 300, 285);
        this.makeInternalFrame("Quadrature Decoders", this.qeiDisplay, 300, 120);
        this.makeInternalFrame("Motors", motDisplay, 300, 200);
        this.makeInternalFrame("Servos", servoDisplay, 300, 300);
        this.makeInternalFrame("DigitalIO", this.digDisplay, 300, 330);
        new StatusPollThread().start();
        this.jf.addWindowListener(new WindowAdapter() {
            public void windowClosing(final WindowEvent e) {
                System.exit(0);
            }
        });
    }
    
    void makeInternalFrame(final String name, final JComponent jc, final int width, final int height) {
        final JInternalFrame jif = new JInternalFrame(name, true, true, true, true);
        jif.setLayout(new BorderLayout());
        jif.add(jc, "Center");
        if (this.ypos + height >= this.jf.getHeight()) {
            this.ypos = 0;
            this.xpos += width;
        }
        jif.reshape(this.xpos, this.ypos, width, height);
        this.ypos += height;
        jif.setVisible(true);
        this.jdp.add(jif);
        this.jdp.setBackground(Color.blue);
    }
    
    public void orcStatus(final Orc orc, final OrcStatus status) {
        if (status.utimeOrc < 2000000L) {
            this.firstStatus = true;
        }
        for (int i = 0; i < 2; ++i) {
            this.qeiDisplay.values[i][1] = String.format("%6d", status.qeiPosition[i]);
            this.qeiDisplay.values[i][2] = String.format("%6d", status.qeiVelocity[i]);
        }
        this.basicDisplay.values[0][1] = String.format("%.6f", status.utimeOrc / 1000000.0);
        for (int i = 0; i < 13; ++i) {
            this.analogDisplay.values[i][1] = String.format("%04X", status.analogInput[i]);
            if (i < 8) {
                double voltage = status.analogInput[i] / 65535.0 * 5.0;
                this.analogDisplay.values[i][2] = String.format("%8.3f V  ", voltage);
                voltage = status.analogInputFiltered[i] / 65535.0 * 5.0;
                this.analogDisplay.values[i][3] = String.format("%8.3f V  ", voltage);
            }
            else if (i >= 8 && i <= 10) {
                double voltage = status.analogInput[i] / 65535.0 * 3.0;
                double current = voltage * 375.0 / 200.0 * 1000.0;
                this.analogDisplay.values[i][2] = String.format("%8.0f mA ", current);
                voltage = status.analogInputFiltered[i] / 65535.0 * 3.0;
                current = voltage * 375.0 / 200.0 * 1000.0;
                this.analogDisplay.values[i][3] = String.format("%8.0f mA ", current);
            }
            else if (i == 11) {
                double voltage = status.analogInput[i] / 65535.0 * 3.0;
                double batvoltage = voltage * 10.1;
                this.analogDisplay.values[i][2] = String.format("%8.2f V  ", batvoltage);
                voltage = status.analogInputFiltered[i] / 65535.0 * 3.0;
                batvoltage = voltage * 10.1;
                this.analogDisplay.values[i][3] = String.format("%8.2f V  ", batvoltage);
            }
            else {
                double voltage = status.analogInput[i] / 65535.0 * 3.0;
                double temp = -(voltage - 2.7) * 75.0 - 55.0;
                this.analogDisplay.values[i][2] = String.format("%8.2f degC", temp);
                voltage = status.analogInputFiltered[i] / 65535.0 * 3.0;
                temp = -(voltage - 2.7) * 75.0 - 55.0;
                this.analogDisplay.values[i][3] = String.format("%8.2f degC", temp);
            }
        }
        for (int i = 0; i < 16; ++i) {
            final int val = status.simpleDigitalValues >> i & 0x1;
            this.digDisplay.values[i][2] = "" + val;
        }
        for (int i = 0; i < 3; ++i) {
            this.motorPanels[i].pwmslider.setActualValue(status.motorPWMactual[i]);
            this.motorPanels[i].slewslider.setActualValue((int)(status.motorSlewSeconds[i] / this.motorPanels[i].slewslider.formatScale));
            this.motorPanels[i].enabledCheckbox.setSelected(status.motorEnable[i]);
            if (this.firstStatus) {
                this.motorPanels[i].pwmslider.setGoalValue(status.motorPWMactual[i]);
                this.motorPanels[i].slewslider.setGoalValue((int)(status.motorSlewSeconds[i] / this.motorPanels[i].slewslider.formatScale));
            }
        }
        this.firstStatus = false;
        this.digDisplay.repaint();
        this.qeiDisplay.repaint();
        this.basicDisplay.repaint();
        this.analogDisplay.repaint();
    }
    
    class StatusPollThread extends Thread
    {
        StatusPollThread() {
            super();
            this.setDaemon(true);
        }
        
        public void run() {
            while (true) {
                try {
                    while (true) {
                        final OrcStatus status = Spy.this.orc.getStatus();
                        Spy.this.orcStatus(Spy.this.orc, status);
                        Thread.sleep(100L);
                    }
                }
                catch (Exception ex) {
                    System.out.println("Spy.StatusPollThread ex: " + ex);
                    continue;
                }
            }
        }
    }
    
    class ServoPanel extends JPanel implements SmallSlider.Listener
    {
        SmallSlider slider;
        Servo servo;
        
        ServoPanel(final int port) {
            super();
            this.slider = new SmallSlider(250, 3750, 1500, 1500, true);
            this.slider.formatScale = 1.0;
            this.slider.formatString = "%.0f us";
            this.servo = new Servo(Spy.this.orc, port, 0.0, 0, 0.0, 0);
            this.setLayout(new BorderLayout());
            this.add(this.slider, "Center");
            this.slider.addListener(this);
        }
        
        public void goalValueChanged(final SmallSlider slider, final int goal) {
            this.servo.setPulseWidth(goal);
        }
    }
    
    class MotorPanel implements SmallSlider.Listener
    {
        SmallSlider pwmslider;
        SmallSlider slewslider;
        JCheckBox enabledCheckbox;
        int port;
        Motor motor;
        
        MotorPanel(final JPanel jp, final int port) {
            super();
            this.pwmslider = new SmallSlider(-255, 255, 0, 0, true);
            this.slewslider = new SmallSlider(0, 65535, 0, 0, true);
            this.enabledCheckbox = new JCheckBox();
            this.motor = new Motor(Spy.this.orc, port, false);
            this.pwmslider.formatScale = 0.39215686274509803;
            this.pwmslider.formatString = "%.0f %%";
            this.slewslider.formatString = "%.3fs";
            this.slewslider.formatScale = 4.5777065690089265E-5;
            jp.add(new JLabel(" " + port));
            jp.add(this.enabledCheckbox);
            jp.add(this.pwmslider);
            jp.add(this.slewslider);
            this.pwmslider.addListener(this);
            this.slewslider.addListener(this);
            this.enabledCheckbox.addActionListener(new ActionListener() {
                public void actionPerformed(final ActionEvent e) {
                    MotorPanel.this.enabledClicked();
                }
            });
        }
        
        public void goalValueChanged(final SmallSlider slider, final int goal) {
            if (slider == this.pwmslider) {
                this.motor.setPWM(goal / 255.0);
            }
            if (slider == this.slewslider) {
                this.motor.setSlewSeconds(goal * this.slewslider.formatScale);
            }
        }
        
        void enabledClicked() {
            if (this.enabledCheckbox.isSelected()) {
                this.motor.setPWM(this.pwmslider.getGoalValue() / 255.0);
            }
            else {
                this.motor.idle();
            }
        }
    }
    
    class TextPanelWidget extends JPanel
    {
        String panelName;
        String[] columnNames;
        String[][] values;
        double[] columnWidthWeight;
        
        public TextPanelWidget(final String[] columnNames, final String[][] values, final double[] columnWidthWeight) {
            super();
            this.columnNames = columnNames;
            this.columnWidthWeight = columnWidthWeight;
            this.values = values;
        }
        
        int getStringWidth(final Graphics g, final String s) {
            final FontMetrics fm = g.getFontMetrics(g.getFont());
            return fm.stringWidth(s);
        }
        
        int getStringHeight(final Graphics g, final String s) {
            final FontMetrics fm = g.getFontMetrics(g.getFont());
            return fm.getMaxAscent();
        }
        
        public Dimension getPreferredSize() {
            return new Dimension(100, 100);
        }
        
        public Dimension getMinimumSize() {
            return this.getPreferredSize();
        }
        
        public Dimension getMaximumSize() {
            return new Dimension(1000, 1000);
        }
        
        public void paint(final Graphics _g) {
            final Graphics2D g = (Graphics2D)_g;
            final int width = this.getWidth();
            final int height = this.getHeight();
            final Font fb = new Font("Monospaced", 1, 12);
            final Font fp = new Font("Monospaced", 0, 12);
            g.setFont(fb);
            final int textHeight = this.getStringHeight(g, "");
            g.setColor(this.getBackground());
            g.fillRect(0, 0, width, height);
            g.setColor(Color.black);
            g.drawRect(0, 0, width - 1, height - 1);
            double totalColumnWidthWeight = 0.0;
            for (int i = 0; i < this.columnWidthWeight.length; ++i) {
                totalColumnWidthWeight += this.columnWidthWeight[i];
            }
            final int[] columnPos = new int[this.columnWidthWeight.length];
            final int[] columnWidth = new int[this.columnWidthWeight.length];
            for (int j = 0; j < columnWidth.length; ++j) {
                columnWidth[j] = (int)(width * this.columnWidthWeight[j] / totalColumnWidthWeight);
            }
            for (int j = 1; j < columnWidth.length; ++j) {
                columnPos[j] = columnPos[j - 1] + columnWidth[j - 1];
            }
            g.setColor(Color.black);
            for (int j = 0; j < this.columnNames.length; ++j) {
                final String s = this.columnNames[j];
                g.drawString(s, columnPos[j] + columnWidth[j] / 2 - this.getStringWidth(g, s) / 2, textHeight * 3 / 2);
            }
            g.setFont(fp);
            for (int j = 0; j < this.values.length; ++j) {
                for (int k = 0; k < this.columnNames.length; ++k) {
                    String s2 = this.values[j][k];
                    if (s2 == null) {
                        s2 = "";
                    }
                    g.drawString(s2, columnPos[k] + columnWidth[k] / 2 - this.getStringWidth(g, s2) / 2, textHeight * (3 + j));
                }
            }
        }
    }
}
