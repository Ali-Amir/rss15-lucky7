package orc.spy;

import javax.swing.*;
import java.awt.geom.*;
import java.awt.*;
import java.util.*;
import java.awt.event.*;

public class SmallSlider extends JComponent
{
    int barheight;
    int minvalue;
    int maxvalue;
    int goalvalue;
    int actualvalue;
    int goalknobsize;
    int actualknobsize;
    int totalheight;
    int marginx;
    ArrayList<Listener> gsls;
    boolean copyactual;
    boolean showactual;
    public String formatString;
    public double formatScale;
    
    public SmallSlider(final int min, final int max, final boolean showactual) {
        super();
        this.barheight = 8;
        this.minvalue = 0;
        this.maxvalue = 100;
        this.goalknobsize = 6;
        this.actualknobsize = 10;
        this.totalheight = this.actualknobsize + 4;
        this.marginx = 6;
        this.gsls = new ArrayList<Listener>();
        this.copyactual = false;
        this.showactual = true;
        this.formatString = "%.0f";
        this.formatScale = 1.0;
        this.minvalue = min;
        this.maxvalue = max;
        this.goalvalue = this.minvalue;
        this.actualvalue = this.minvalue;
        this.showactual = showactual;
        this.addMouseMotionListener(new SmallSliderMouseMotionListener());
        this.addMouseListener(new SmallSliderMouseMotionListener());
        this.copyactual = true;
    }
    
    public SmallSlider(final int min, final int max, final int goalvalue, final int actualvalue, final boolean showactual) {
        super();
        this.barheight = 8;
        this.minvalue = 0;
        this.maxvalue = 100;
        this.goalknobsize = 6;
        this.actualknobsize = 10;
        this.totalheight = this.actualknobsize + 4;
        this.marginx = 6;
        this.gsls = new ArrayList<Listener>();
        this.copyactual = false;
        this.showactual = true;
        this.formatString = "%.0f";
        this.formatScale = 1.0;
        this.minvalue = min;
        this.maxvalue = max;
        this.goalvalue = goalvalue;
        this.actualvalue = actualvalue;
        this.showactual = showactual;
        this.addMouseMotionListener(new SmallSliderMouseMotionListener());
        this.addMouseListener(new SmallSliderMouseMotionListener());
    }
    
    public void addListener(final Listener gsl) {
        this.gsls.add(gsl);
    }
    
    public void setMaximum(final int i) {
        this.maxvalue = i;
        this.repaint();
    }
    
    public void setMinimum(final int i) {
        this.minvalue = i;
        this.repaint();
    }
    
    public synchronized void setActualValue(final int i) {
        if (this.copyactual) {
            this.goalvalue = i;
            this.copyactual = false;
        }
        this.actualvalue = i;
        this.repaint();
    }
    
    public void setGoalValue(final int i) {
        this.goalvalue = i;
        this.repaint();
    }
    
    public int getGoalValue() {
        return this.goalvalue;
    }
    
    public int getActualValue() {
        return this.actualvalue;
    }
    
    public Dimension getMinimumSize() {
        return new Dimension(40, this.totalheight);
    }
    
    public Dimension getPreferredSize() {
        return this.getMinimumSize();
    }
    
    public synchronized void paint(final Graphics gin) {
        final Graphics2D g = (Graphics2D)gin;
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        final int height = this.getHeight();
        final int width = this.getWidth() - 2 * this.marginx;
        final int cy = height / 2;
        final int cx = width / 2;
        g.translate(this.marginx, 0);
        g.setColor(this.getParent().getBackground());
        g.fillRect(0, 0, width, height);
        final RoundRectangle2D.Double barr = new RoundRectangle2D.Double(0.0, cy - this.barheight / 2, width, this.barheight, this.barheight, this.barheight);
        g.setColor(Color.white);
        g.fill(barr);
        g.setColor(Color.black);
        g.draw(barr);
        int x = width * (this.goalvalue - this.minvalue) / (this.maxvalue - this.minvalue);
        final Ellipse2D.Double goalknob = new Ellipse2D.Double(x - this.goalknobsize / 2, cy - this.goalknobsize / 2, this.goalknobsize, this.goalknobsize);
        g.setColor(Color.green);
        g.fill(goalknob);
        g.setStroke(new BasicStroke(1.0f));
        g.setColor(Color.black);
        g.draw(goalknob);
        g.setFont(new Font("Monospaced", 0, 11));
        if (this.showactual) {
            x = width * (this.actualvalue - this.minvalue) / (this.maxvalue - this.minvalue);
            g.setColor(Color.black);
            g.setStroke(new BasicStroke(1.0f));
            final Ellipse2D.Double actualknob = new Ellipse2D.Double(x - this.actualknobsize / 2, cy - this.actualknobsize / 2, this.actualknobsize, this.actualknobsize);
            g.draw(actualknob);
        }
        g.setColor(Color.black);
        final String s = String.format(this.formatString, this.formatScale * this.goalvalue);
        g.drawString(s, width - s.length() * 8, cy + 16);
    }
    
    void handleClick(final int x) {
        this.goalvalue = this.minvalue + (this.maxvalue - this.minvalue) * (x - this.marginx) / (this.getWidth() - 2 * this.marginx);
        if (this.goalvalue < this.minvalue) {
            this.goalvalue = this.minvalue;
        }
        if (this.goalvalue > this.maxvalue) {
            this.goalvalue = this.maxvalue;
        }
        for (final Listener gsl : this.gsls) {
            gsl.goalValueChanged(this, this.goalvalue);
        }
        this.repaint();
    }
    
    class SmallSliderMouseMotionListener implements MouseMotionListener, MouseListener
    {
        public void mouseDragged(final MouseEvent e) {
            SmallSlider.this.handleClick(e.getX());
        }
        
        public void mouseMoved(final MouseEvent e) {
        }
        
        public void mouseClicked(final MouseEvent e) {
            SmallSlider.this.handleClick(e.getX());
        }
        
        public void mouseEntered(final MouseEvent e) {
        }
        
        public void mouseExited(final MouseEvent e) {
        }
        
        public void mousePressed(final MouseEvent e) {
            SmallSlider.this.handleClick(e.getX());
        }
        
        public void mouseReleased(final MouseEvent e) {
        }
    }
    
    public interface Listener
    {
        void goalValueChanged(SmallSlider p0, int p1);
    }
}
