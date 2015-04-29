package orc;

import orc.spy.*;
import java.awt.event.*;
import java.awt.*;
import java.net.*;
import javax.swing.*;
import javax.swing.event.*;
import java.util.*;
import java.io.*;

public class OrcSetup
{
    InetAddress broadcastInetAddr;
    OrcDetection selectedOrcDetection;
    ArrayList<OrcDetection> detections;
    JFrame jf;
    HashMap<String, Param> paramsMap;
    JPanel paramsPanel;
    JButton revertButton;
    JButton randomMACButton;
    JButton writeButton;
    JTextArea log;
    MyListModel mdl;
    JList orcList;
    ArrayList<String> orcIps;
    Random rand;
    FindOrcThread finder;
    DatagramSocket sock;
    static final int FLASH_PARAM_SIGNATURE = -311321921;
    static final int FLASH_PARAM_VERSION = 2;
    static final int FLASH_PARAM_LENGTH = 37;
    
    public static void main(final String[] args) {
        new OrcSetup();
    }
    
    void addParamIPAddress(final String name) {
        this.paramsPanel.add(new JLabel(name));
        final Param p = new StringParam();
        this.paramsPanel.add(p.getComponent());
        this.paramsMap.put(name, p);
    }
    
    void addParamMACAddress(final String name) {
        this.paramsPanel.add(new JLabel(name));
        final Param p = new StringParam();
        this.paramsPanel.add(p.getComponent());
        this.paramsMap.put(name, p);
    }
    
    void addParamBoolean(final String name) {
        this.paramsPanel.add(new JLabel(name));
        final Param p = new BooleanParam();
        this.paramsPanel.add(p.getComponent());
        this.paramsMap.put(name, p);
    }
    
    public OrcSetup() {
        super();
        this.selectedOrcDetection = null;
        this.detections = new ArrayList<OrcDetection>();
        this.paramsMap = new HashMap<String, Param>();
        this.revertButton = new JButton("Revert");
        this.randomMACButton = new JButton("Randomize MAC");
        this.writeButton = new JButton("Write to uOrc");
        this.log = new JTextArea();
        this.mdl = new MyListModel();
        this.orcList = new JList(this.mdl);
        this.orcIps = new ArrayList<String>();
        this.rand = new Random();
        try {
            this.broadcastInetAddr = InetAddress.getByName("192.168.237.255");
        }
        catch (UnknownHostException ex) {
            System.out.println("ex: " + ex);
            System.exit(-1);
        }
        (this.finder = new FindOrcThread()).start();
        this.orcList.addListSelectionListener(new ListSelectionListener() {
            public void valueChanged(final ListSelectionEvent e) {
                OrcSetup.this.listChanged();
            }
        });
        (this.jf = new JFrame("OrcSetup")).setLayout(new BorderLayout());
        this.paramsPanel = new JPanel();
        final WeightedGridLayout wgl = new WeightedGridLayout(new double[] { 0.1, 0.9 });
        wgl.setDefaultRowWeight(0.0);
        this.paramsPanel.setLayout(wgl);
        this.addParamIPAddress("ipaddr");
        this.addParamIPAddress("ipmask");
        this.addParamIPAddress("ipgw");
        this.addParamMACAddress("macaddr");
        this.addParamBoolean("dhcpd_enable");
        final JPanel buttonPanel = new JPanel();
        buttonPanel.setLayout(new FlowLayout());
        buttonPanel.add(this.revertButton);
        buttonPanel.add(this.randomMACButton);
        buttonPanel.add(this.writeButton);
        final JPanel mainPanel = new JPanel();
        mainPanel.setLayout(new BorderLayout());
        mainPanel.add(this.paramsPanel, "Center");
        mainPanel.add(buttonPanel, "South");
        final JPanel orcListPanel = new JPanel();
        orcListPanel.setLayout(new BorderLayout());
        orcListPanel.add(new JLabel("uOrcs found:"), "North");
        orcListPanel.add(new JScrollPane(this.orcList), "Center");
        orcListPanel.add(Box.createHorizontalStrut(10), "East");
        mainPanel.add(orcListPanel, "West");
        final JSplitPane jsp = new JSplitPane(0, mainPanel, new JScrollPane(this.log));
        this.jf.add(jsp, "Center");
        jsp.setDividerLocation(0.65);
        jsp.setResizeWeight(0.65);
        this.revertButton.addActionListener(new ActionListener() {
            public void actionPerformed(final ActionEvent e) {
                OrcSetup.this.revert();
            }
        });
        this.randomMACButton.addActionListener(new ActionListener() {
            public void actionPerformed(final ActionEvent e) {
                OrcSetup.this.randomMAC();
            }
        });
        this.writeButton.addActionListener(new ActionListener() {
            public void actionPerformed(final ActionEvent e) {
                OrcSetup.this.write();
            }
        });
        this.jf.setSize(600, 500);
        this.jf.setVisible(true);
        this.log.setEditable(false);
        this.log.setFont(new Font("Monospaced", 0, 12));
        this.logAppend("Started.");
        this.listChanged();
    }
    
    void listChanged() {
        final int idx = this.orcList.getSelectedIndex();
        if (idx < 0) {
            this.paramsMap.get("ipaddr").setValue("");
            this.paramsMap.get("ipmask").setValue("");
            this.paramsMap.get("ipgw").setValue("");
            this.paramsMap.get("macaddr").setValue("");
            this.revertButton.setEnabled(false);
            this.randomMACButton.setEnabled(false);
            this.writeButton.setEnabled(false);
            return;
        }
        this.selectedOrcDetection = this.detections.get(idx);
        this.paramsMap.get("ipaddr").setValue(this.ip2string(this.selectedOrcDetection.ip4addr));
        this.paramsMap.get("ipmask").setValue(this.ip2string(this.selectedOrcDetection.ip4mask));
        this.paramsMap.get("ipgw").setValue(this.ip2string(this.selectedOrcDetection.ip4gw));
        this.paramsMap.get("macaddr").setValue(this.mac2string(this.selectedOrcDetection.macaddr));
        this.paramsMap.get("dhcpd_enable").setValue(this.boolean2string(this.selectedOrcDetection.dhcpd_enable));
        this.revertButton.setEnabled(true);
        this.randomMACButton.setEnabled(true);
        this.writeButton.setEnabled(true);
    }
    
    void revert() {
        this.listChanged();
    }
    
    void randomMAC() {
        final long r = this.rand.nextLong();
        final Param p = this.paramsMap.get("macaddr");
        p.setValue(String.format("%02x:%02x:%02x:%02x:%02x:%02x", 2, 0, r >> 24 & 0xFFL, r >> 16 & 0xFFL, r >> 8 & 0xFFL, r & 0xFFL));
    }
    
    void logAppend(final String s) {
        this.log.append(s);
        this.log.setCaretPosition(this.log.getText().length() - 1);
    }
    
    int swap(final int v) {
        return ((v >> 24 & 0xFF) << 0) + ((v >> 16 & 0xFF) << 8) + ((v >> 8 & 0xFF) << 16) + ((v & 0xFF) << 24);
    }
    
    long swap(final long v) {
        return ((v >> 56 & 0xFFL) << 0) + ((v >> 48 & 0xFFL) << 8) + ((v >> 40 & 0xFFL) << 16) + ((v >> 32 & 0xFFL) << 24) + ((v >> 24 & 0xFFL) << 32) + ((v >> 16 & 0xFFL) << 40) + ((v >> 8 & 0xFFL) << 48) + ((v & 0xFFL) << 56);
    }
    
    int string2ip(final String s) {
        final String[] toks = s.split("\\.");
        if (toks.length != 4) {
            throw new RuntimeException("Invalid IP format");
        }
        return (Integer.parseInt(toks[0]) << 24) + (Integer.parseInt(toks[1]) << 16) + (Integer.parseInt(toks[2]) << 8) + Integer.parseInt(toks[3]);
    }
    
    String ip2string(final int ip) {
        return String.format("%d.%d.%d.%d", ip >> 24 & 0xFF, ip >> 16 & 0xFF, ip >> 8 & 0xFF, ip & 0xFF);
    }
    
    long string2mac(final String s) {
        final String[] toks = s.split(":");
        if (toks.length != 6) {
            throw new RuntimeException("Invalid MAC address format");
        }
        return (Integer.parseInt(toks[0], 16) << 40) + (Integer.parseInt(toks[1], 16) << 32) + (Integer.parseInt(toks[2], 16) << 24) + (Integer.parseInt(toks[3], 16) << 16) + (Integer.parseInt(toks[4], 16) << 8) + (Integer.parseInt(toks[5], 16) << 0);
    }
    
    boolean string2boolean(String s) {
        s = s.toLowerCase();
        return s.equals("true");
    }
    
    String boolean2string(final boolean b) {
        if (b) {
            return "true";
        }
        return "false";
    }
    
    String mac2string(final long mac) {
        return String.format("%02x:%02x:%02x:%02x:%02x:%02x", mac >> 40 & 0xFFL, mac >> 32 & 0xFFL, mac >> 24 & 0xFFL, mac >> 16 & 0xFFL, mac >> 8 & 0xFFL, mac & 0xFFL);
    }
    
    void logArray(final byte[] buf, final int offset, final int len) {
        for (int i = 0; i < len - offset; ++i) {
            if (i % 16 == 0) {
                this.logAppend(String.format("%04x : ", i));
            }
            this.logAppend(String.format("%02x ", buf[offset + i]));
            if (i % 16 == 15) {
                this.logAppend("\n");
            }
        }
        this.logAppend("\n");
    }
    
    void write() {
        try {
            final ByteArrayOutputStream bouts = new ByteArrayOutputStream();
            final DataOutputStream outs = new DataOutputStream(bouts);
            final OrcDetection od = this.detections.get(this.orcList.getSelectedIndex());
            outs.writeInt(-307579079);
            outs.writeInt(65281);
            outs.writeInt(od.bootNonce);
            outs.writeInt(128);
            outs.writeInt(this.swap(-311321921));
            outs.writeInt(this.swap(2));
            outs.writeInt(this.swap(37));
            outs.writeInt(this.swap(this.string2ip(this.paramsMap.get("ipaddr").getValue())));
            outs.writeInt(this.swap(this.string2ip(this.paramsMap.get("ipmask").getValue())));
            outs.writeInt(this.swap(this.string2ip(this.paramsMap.get("ipgw").getValue())));
            outs.writeLong(this.swap(this.string2mac(this.paramsMap.get("macaddr").getValue())));
            outs.writeByte(this.string2boolean(this.paramsMap.get("dhcpd_enable").getValue()) ? 1 : 0);
            outs.writeInt(this.swap(-311321921));
            this.logAppend("Writing parameter block...\n");
            final byte[] p = bouts.toByteArray();
            System.out.println(od.addr);
            final DatagramPacket packet = new DatagramPacket(p, p.length, this.broadcastInetAddr, 2377);
            this.sock.send(packet);
            this.logAppend("...finished. Reset uORC for settings to take effect.\n\n");
        }
        catch (IOException ex) {
            System.out.println("ex: " + ex);
        }
    }
    
    class OrcDetection implements Comparable<OrcDetection>
    {
        InetAddress addr;
        long mstime;
        int bootNonce;
        int magic;
        int version;
        int length;
        int ip4addr;
        int ip4mask;
        int ip4gw;
        long macaddr;
        boolean dhcpd_enable;
        int magic2;
        
        public int compareTo(final OrcDetection od) {
            if (od.ip4addr == this.ip4addr) {
                return od.bootNonce - this.bootNonce;
            }
            return od.ip4addr - this.ip4addr;
        }
    }
    
    class BooleanParam implements Param
    {
        JCheckBox checkBox;
        
        BooleanParam() {
            super();
            this.checkBox = new JCheckBox();
        }
        
        public String getValue() {
            return this.checkBox.isSelected() ? "true" : "false";
        }
        
        public void setValue(final String v) {
            this.checkBox.setSelected(v.equals("true"));
        }
        
        public JComponent getComponent() {
            return this.checkBox;
        }
    }
    
    class StringParam implements Param
    {
        JTextField textField;
        
        StringParam() {
            super();
            this.textField = new JTextField();
        }
        
        public String getValue() {
            return this.textField.getText();
        }
        
        public void setValue(final String v) {
            this.textField.setText(v);
        }
        
        public JComponent getComponent() {
            return this.textField;
        }
    }
    
    class MyListModel implements ListModel
    {
        ArrayList<ListDataListener> listeners;
        
        MyListModel() {
            super();
            this.listeners = new ArrayList<ListDataListener>();
        }
        
        public void addListDataListener(final ListDataListener listener) {
            this.listeners.add(listener);
        }
        
        public Object getElementAt(final int index) {
            final OrcDetection od = OrcSetup.this.detections.get(index);
            return String.format("%08x : %s", od.bootNonce, OrcSetup.this.ip2string(od.ip4addr));
        }
        
        public int getSize() {
            return OrcSetup.this.detections.size();
        }
        
        public void removeListDataListener(final ListDataListener listener) {
            this.listeners.remove(listener);
        }
        
        public void changed() {
            for (final ListDataListener listener : this.listeners) {
                listener.contentsChanged(new ListDataEvent(this, 0, 0, this.getSize()));
            }
        }
    }
    
    class FindOrcThread extends Thread
    {
        HashMap<Integer, OrcDetection> detectionsMap;
        ReaderThread reader;
        
        public FindOrcThread() {
            super();
            this.detectionsMap = new HashMap<Integer, OrcDetection>();
            this.setDaemon(true);
        }
        
        public void run() {
            try {
                this.runEx();
            }
            catch (IOException ex) {
                System.out.println("Ex: " + ex);
            }
            catch (InterruptedException ex2) {
                System.out.println("Ex: " + ex2);
            }
        }
        
        void runEx() throws IOException, InterruptedException {
            OrcSetup.this.sock = new DatagramSocket(2377);
            (this.reader = new ReaderThread()).setDaemon(true);
            this.reader.start();
            while (true) {
                final ByteArrayOutputStream bouts = new ByteArrayOutputStream();
                final DataOutputStream outs = new DataOutputStream(bouts);
                outs.writeInt(-307579079);
                outs.writeInt(0);
                final byte[] p = bouts.toByteArray();
                final DatagramPacket packet = new DatagramPacket(p, p.length, OrcSetup.this.broadcastInetAddr, 2377);
                OrcSetup.this.sock.send(packet);
                Thread.sleep(500L);
                final ArrayList<OrcDetection> goodDetections = new ArrayList<OrcDetection>();
                synchronized (this.detectionsMap) {
                    for (final OrcDetection od : this.detectionsMap.values()) {
                        final double age = (System.currentTimeMillis() - od.mstime) / 1000.0;
                        if (age < 1.2) {
                            goodDetections.add(od);
                        }
                    }
                    this.detectionsMap.clear();
                    for (final OrcDetection od : goodDetections) {
                        this.detectionsMap.put(od.bootNonce, od);
                    }
                }
                Collections.sort(goodDetections);
                boolean changed = false;
                if (goodDetections.size() != OrcSetup.this.detections.size()) {
                    changed = true;
                }
                if (!changed) {
                    for (int i = 0; i < goodDetections.size(); ++i) {
                        if (goodDetections.get(i) != OrcSetup.this.detections.get(i)) {
                            changed = true;
                        }
                    }
                }
                if (changed) {
                    OrcSetup.this.detections = goodDetections;
                    if (OrcSetup.this.selectedOrcDetection != null && !OrcSetup.this.detections.contains(OrcSetup.this.selectedOrcDetection)) {
                        OrcSetup.this.orcList.clearSelection();
                        OrcSetup.this.listChanged();
                    }
                    OrcSetup.this.mdl.changed();
                }
            }
        }
        
        class ReaderThread extends Thread
        {
            public void run() {
                try {
                    this.runEx();
                }
                catch (IOException ex) {
                    System.out.println("Ex: " + ex);
                }
            }
            
            void runEx() throws IOException {
                while (true) {
                    final byte[] buf = new byte[1600];
                    final DatagramPacket packet = new DatagramPacket(buf, buf.length);
                    OrcSetup.this.sock.receive(packet);
                    final DataInputStream ins = new DataInputStream(new ByteArrayInputStream(buf, 0, packet.getLength()));
                    if (ins.available() < 4) {
                        continue;
                    }
                    final int magic = ins.readInt();
                    if (magic != 1968132675) {
                        continue;
                    }
                    final int bootNonce = ins.readInt();
                    OrcDetection od = FindOrcThread.this.detectionsMap.get(bootNonce);
                    if (od == null) {
                        od = new OrcDetection();
                        od.bootNonce = bootNonce;
                        FindOrcThread.this.detectionsMap.put(bootNonce, od);
                    }
                    od.addr = packet.getAddress();
                    od.mstime = System.currentTimeMillis();
                    od.magic = OrcSetup.this.swap(ins.readInt());
                    od.version = OrcSetup.this.swap(ins.readInt());
                    od.length = OrcSetup.this.swap(ins.readInt());
                    od.ip4addr = OrcSetup.this.swap(ins.readInt());
                    od.ip4mask = OrcSetup.this.swap(ins.readInt());
                    od.ip4gw = OrcSetup.this.swap(ins.readInt());
                    od.macaddr = OrcSetup.this.swap(ins.readLong());
                    od.dhcpd_enable = (ins.readByte() != 0);
                    od.magic2 = OrcSetup.this.swap(ins.readInt());
                }
            }
        }
    }
    
    interface Param
    {
        String getValue();
        
        void setValue(String p0);
        
        JComponent getComponent();
    }
}
