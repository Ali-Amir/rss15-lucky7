package orc.examples;

import java.util.*;
import javax.swing.*;
import javax.swing.border.*;
import java.awt.*;
import javax.swing.event.*;
import orc.*;

public class AX12Demo
{
    JFrame jf;
    ArrayList<AX12Servo> servos;
    
    public static void main(final String[] args) {
        final Orc orc = Orc.makeOrc();
        orc.verbose = true;
        System.out.printf("Checking communication with Orc board...", new Object[0]);
        System.out.flush();
        final OrcStatus os = orc.getStatus();
        System.out.println("good!");
        System.out.println("Scanning for AX12 servos...");
        final ArrayList<AX12Servo> servos = new ArrayList<AX12Servo>();
        for (int id = 0; id < 254; ++id) {
            System.out.printf("%4d\r", id);
            System.out.flush();
            final AX12Servo servo = new AX12Servo(orc, id);
            if (servo.ping()) {
                System.out.printf("%d  found!\n", id);
                servos.add(servo);
            }
        }
        if (servos.size() == 0) {
            System.out.println("No AX12 servos found.");
            System.exit(0);
        }
        new AX12Demo(servos);
    }
    
    public AX12Demo(final ArrayList<AX12Servo> servos) {
        super();
        this.jf = new JFrame("AX12Demo");
        this.servos = servos;
        this.jf.setLayout(new GridLayout(servos.size(), 1));
        for (final AX12Servo s : servos) {
            this.jf.add(new AX12Widget(s));
        }
        this.jf.setSize(800, 400);
        this.jf.setVisible(true);
    }
    
    class AX12Widget extends JPanel implements ChangeListener
    {
        AX12Servo servo;
        JSlider js;
        JLabel positionLabel;
        JLabel speedLabel;
        JLabel loadLabel;
        JLabel voltageLabel;
        JLabel tempLabel;
        JLabel errorsLabel;
        JSlider positionSlider;
        
        public AX12Widget(final AX12Servo servo) {
            super();
            this.positionLabel = new JLabel("");
            this.speedLabel = new JLabel("");
            this.loadLabel = new JLabel("");
            this.voltageLabel = new JLabel("");
            this.tempLabel = new JLabel("");
            this.errorsLabel = new JLabel("");
            this.positionSlider = new JSlider(0, 300, 150);
            this.servo = servo;
            this.setBorder(BorderFactory.createTitledBorder("AX12 id " + servo.getId()));
            this.setLayout(new BorderLayout());
            final JPanel jp = new JPanel();
            jp.setLayout(new GridLayout(6, 2));
            jp.add(new JLabel("Position"));
            jp.add(this.positionLabel);
            jp.add(new JLabel("Speed"));
            jp.add(this.speedLabel);
            jp.add(new JLabel("Load"));
            jp.add(this.loadLabel);
            jp.add(new JLabel("Voltage"));
            jp.add(this.voltageLabel);
            jp.add(new JLabel("Temp. (C)  "));
            jp.add(this.tempLabel);
            jp.add(new JLabel("Errors"));
            jp.add(this.errorsLabel);
            this.add(jp, "East");
            this.add(this.positionSlider, "Center");
            this.positionSlider.addChangeListener(this);
            new RunThread().start();
        }
        
        public void stateChanged(final ChangeEvent e) {
            if (e.getSource() == this.positionSlider) {
                this.servo.setGoalDegrees(this.positionSlider.getValue(), 0.3, 0.3);
            }
        }
        
        class RunThread extends Thread
        {
            public void run() {
                while (true) {
                    final AX12Status status = AX12Widget.this.servo.getStatus();
                    AX12Widget.this.positionLabel.setText(String.format("%.3f", status.positionDegrees));
                    AX12Widget.this.speedLabel.setText(String.format("%.3f", status.speed));
                    AX12Widget.this.loadLabel.setText(String.format("%.3f", status.load));
                    AX12Widget.this.voltageLabel.setText(String.format("%.3f", status.voltage));
                    AX12Widget.this.errorsLabel.setText(String.format("%02x", status.error_flags));
                    AX12Widget.this.tempLabel.setText(String.format("%.1f", status.temperature));
                    try {
                        Thread.sleep(50L);
                    }
                    catch (InterruptedException ex) {}
                }
            }
        }
    }
}
