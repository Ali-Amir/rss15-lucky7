package orc.spy;

import java.util.*;
import java.awt.*;

public class WeightedGridLayout implements LayoutManager
{
    int hgap;
    int vgap;
    double[] columnWeights;
    int cols;
    int rows;
    HashMap<Integer, Double> rowWeightsMap;
    double defaultRowWeight;
    double[] rowWeights;
    int minimumWidth;
    int minimumHeight;
    int[] minimumColumnWidth;
    int[] minimumRowHeight;
    
    public WeightedGridLayout(final double[] columnWeights, final int hgap, final int vgap) {
        super();
        this.rowWeightsMap = new HashMap<Integer, Double>();
        this.defaultRowWeight = 0.0;
        this.rowWeights = null;
        this.minimumWidth = 0;
        this.minimumHeight = 0;
        this.hgap = hgap;
        this.vgap = vgap;
        this.columnWeights = this.normalizeSumToOne(columnWeights);
        this.cols = columnWeights.length;
    }
    
    public WeightedGridLayout(final double[] columnWeights) {
        this(columnWeights, 4, 4);
    }
    
    public void setDefaultRowWeight(final double v) {
        this.defaultRowWeight = v;
    }
    
    public void setRowWeight(final int row, final double v) {
        this.rowWeightsMap.put(row, v);
    }
    
    public void addLayoutComponent(final String name, final Component comp) {
    }
    
    Component getComponentXY(final Container parent, final int x, final int y) {
        return parent.getComponents()[x + y * this.cols];
    }
    
    int sum(final int[] v) {
        int acc = 0;
        for (int i = 0; i < v.length; ++i) {
            acc += v[i];
        }
        return acc;
    }
    
    double[] normalizeSumToOne(final double[] v) {
        double acc = 0.0;
        for (int i = 0; i < v.length; ++i) {
            acc += v[i];
        }
        final double[] res = new double[v.length];
        for (int j = 0; j < v.length; ++j) {
            res[j] = ((acc == 0.0) ? (1.0 / v.length) : (v[j] / acc));
        }
        return res;
    }
    
    void computeGeometry(final Container parent) {
        this.rows = parent.getComponents().length / this.cols;
        this.rowWeights = new double[this.rows];
        for (int i = 0; i < this.rowWeights.length; ++i) {
            if (this.rowWeightsMap.get(i) == null) {
                this.rowWeights[i] = this.defaultRowWeight;
            }
            else {
                this.rowWeights[i] = this.rowWeightsMap.get(i);
            }
        }
        this.rowWeights = this.normalizeSumToOne(this.rowWeights);
        this.minimumColumnWidth = new int[this.cols];
        this.minimumRowHeight = new int[this.rows];
        for (int col = 0; col < this.cols; ++col) {
            for (int row = 0; row < this.rows; ++row) {
                final Dimension thisdim = this.getComponentXY(parent, col, row).getMinimumSize();
                this.minimumColumnWidth[col] = (int)Math.max(this.minimumColumnWidth[col], thisdim.getWidth());
                this.minimumRowHeight[row] = (int)Math.max(this.minimumRowHeight[row], thisdim.getHeight());
            }
        }
        this.minimumWidth = this.sum(this.minimumColumnWidth) + this.hgap * (this.cols - 1);
        this.minimumHeight = this.sum(this.minimumRowHeight) + this.vgap * (this.rows - 1);
    }
    
    public void layoutContainer(final Container parent) {
        this.computeGeometry(parent);
        final int extraWidth = parent.getWidth() - this.minimumWidth;
        final int extraHeight = parent.getHeight() - this.minimumHeight;
        final int[] columnWidth = new int[this.cols];
        for (int col = 0; col < this.cols; ++col) {
            columnWidth[col] = (int)(this.minimumColumnWidth[col] + this.columnWeights[col] * extraWidth);
        }
        final int[] rowHeight = new int[this.rows];
        for (int row = 0; row < this.rows; ++row) {
            rowHeight[row] = (int)(this.minimumRowHeight[row] + this.rowWeights[row] * extraHeight);
        }
        final int[] columnPosition = new int[this.cols];
        for (int col2 = 1; col2 < this.cols; ++col2) {
            final int[] array = columnPosition;
            final int n = col2;
            array[n] += columnPosition[col2 - 1] + columnWidth[col2 - 1];
        }
        final int[] rowPosition = new int[this.rows];
        for (int row2 = 1; row2 < this.rows; ++row2) {
            final int[] array2 = rowPosition;
            final int n2 = row2;
            array2[n2] += rowPosition[row2 - 1] + rowHeight[row2 - 1];
        }
        for (int row2 = 0; row2 < this.rows; ++row2) {
            for (int col3 = 0; col3 < this.cols; ++col3) {
                final Component c = this.getComponentXY(parent, col3, row2);
                c.setSize(new Dimension(columnWidth[col3], rowHeight[row2]));
                c.setLocation(new Point(columnPosition[col3], rowPosition[row2]));
            }
        }
    }
    
    public Dimension minimumLayoutSize(final Container parent) {
        this.computeGeometry(parent);
        return new Dimension(this.minimumWidth, this.minimumHeight);
    }
    
    public Dimension preferredLayoutSize(final Container parent) {
        return this.minimumLayoutSize(parent);
    }
    
    public void removeLayoutComponent(final Component comp) {
    }
}
