package com.github.rosjava.challenge.vision;

import java.awt.Color;
import com.github.rosjava.challenge.gui.Image;

/**
 * BlobTracking performs image processing and tracking for the VisualServo
 * module.  BlobTracking filters raw pixels from an image and classifies blobs,
 * generating a higher-level vision image.
 *
 * @author previous TA's, prentice
 */
public class BlobTracking {
	protected int stepCounter = 0;
	protected double lastStepTime = 0.0;

	public int width;
	public int height;

	public double rectThreshold = 1.4;

	protected float histogram[][]; //(Solution)
	// (Solution)
	protected int blobMask[] = null; //(Solution)
	protected int blobPixelRedMask[] = null; //(Solution)
	protected int blobPixelBlueMask[] = null; //(Solution)
	protected int blobPixelGreenMask[] = null; //(Solution)
	protected int blobPixelYellowMask[] = null; //(Solution)
	protected int imageFiltered[] = null; //(Solution)
	protected int imageConnected[] = null; //(Solution)
	protected float imageHsb[] = null; //(Solution)
	// (Solution)
	public double targetRedHueLevel=0.0; // (Solution)
	public double targetBlueHueLevel=0.66; 
	public double targetGreenHueLevel=0.33; 
	public double targetYellowHueLevel=0.167;

	public double max_area = -1;

	public double targetRadius=28; // (Solution)
	public double hueThreshold=0.05; // (Solution)
	public double saturationLevel=0.6; // (Solution)
	public double blobSizeThreshold=400.0/128.0/128.0; // (Solution)
	public double desiredFixationDistance=0.4; // (Solution)
	public double translationErrorTolerance=0.05; // (Solution)
	public double rotationErrorTolerance=Math.PI/180.0*1.0; // (Solution)
	public boolean useGaussianBlur=false; // (Solution)
	public boolean approximateGaussian=false; // (Solution)
	public double translationVelocityGain=0.8; // (Solution)
	public double translationVelocityMax=0.1; // (Solution)
	public double rotationVelocityGain=0.9; // (Solution)
	public double rotationVelocityMax=Math.PI/10.0; // (Solution)
	// (Solution)
	/** //(Solution)
	 * <p>Focal plane distance, experimentally determined, in pixels.</p> //(Solution)
	 **/ //(Solution)
	public double focalPlaneDistance = 107.0; //(Solution)
	// (Solution)
	protected ConnectedComponents connComp = new ConnectedComponents(); //(Solution)
	// (Solution)
	public double translationVelocityCommand = 0.0; // (Solution)
	public double rotationVelocityCommand = 0.0; // (Solution)

	// Variables used for velocity controller that are available to calling
	// process.  Visual results are valid only if targetDetected==true; motor
	// velocities should do something sane in this case.
	public boolean targetDetected = false; // set in blobPresent()
	public double centroidX = 0.0; // set in blobPresent()
	public double centroidY = 0.0; // set in blobPresent()
	public double targetArea = 0.0; // set in blobPresent()
	public double targetRange = 0.0; // set in blobFix()
	public double targetBearing = 0.0; // set in blobFix()

	/**
	 * <p>Create a BlobTracking object</p>
	 *
	 * @param width image width
	 * @param height image height
	 */
	public BlobTracking(int width, int height) {

		this.width = width;
		this.height = height;

		histogram = new float[width][3]; // (Solution)
		// (Solution)
		// initialize image structures and masks // (Solution)
		blobMask = new int[width * height]; // (Solution)
		blobPixelRedMask = new int[width * height]; // (Solution)
		blobPixelBlueMask = new int[width * height]; // (Solution)
		blobPixelGreenMask = new int[width * height]; // (Solution)
		blobPixelYellowMask = new int[width * height]; // (Solution)
		imageConnected = new int[width * height]; // (Solution)
		imageHsb = new float[width * height * 3]; // (Solution)
	}

	//(Solution)
	//(Solution)
	/** // (Solution)
	 * <p>Apply connected components analysis to pick out the largest blob. Then // (Solution)
	 * build stats on this blob.</p> // (Solution)
	 **/ // (Solution)
	protected synchronized void blobPresent(int[] threshIm, int[] connIm, int[] blobIm) { // (Solution)
    int arraySize = width*height;
    connIm = connComp.doLabel(threshIm, connIm, width, height);

    int numLabels = connComp.getNumberOfLabels();
    int[] labelArea = new int[numLabels];
    int[] minX = new int[numLabels];
    int[] maxX = new int[numLabels];
    int[] minY = new int[numLabels];
    int[] maxY = new int[numLabels];
    int[] sumX = new int[numLabels];
    int[] sumY = new int[numLabels];

    for (int i = 0; i < numLabels; ++i) {
      minX[i] = width;
      minY[i] = height;
    }
    for (int i = 0; i < arraySize; ++i) {
      int x = i % width;
      int y = i / width;
      if (connIm[i] != 0) {
        int l = connIm[i]-1;
        ++labelArea[l];

        minX[l] = Math.min(minX[l], x);
        maxX[l] = Math.max(maxX[l], x);
        minY[l] = Math.min(minY[l], y);
        maxY[l] = Math.max(maxY[l], y);
        sumX[l] += x;
        sumY[l] += y;
      }
    }

    targetDetected = false;
    boolean foundAtLeastOne = false;
    for (int i = 0; i < numLabels; ++i) {
      int b_w = maxX[i] - minX[i] + 1;
      int b_h = maxY[i] - minY[i] + 1;
      int r = (b_w + b_h) / 4;
      double minArea = Math.min(Math.PI*r*r, labelArea[i]*1.0);
      double maxArea = Math.max(Math.PI*r*r, labelArea[i]*1.0);
      double aspectRatio = 1.0*b_w/b_h;
      if (r > 5) {
        System.out.println("HERE: r="+ r + " minx=" + minX[i] + " maxx=" + maxX[i]
                            + " miny=" + minY[i] + " maxy=" + maxY[i]);
      }
      if (Math.abs(1.0 - aspectRatio) < 0.2 && r > 5) {



      	if (b_w*b_h>max_area && !isDouble(b_w, b_h)){
	        int xC = (minX[i] + maxX[i]) / 2;
	        int yC = (minY[i] + maxY[i]) / 2;


	        // Set internal variables
	        targetDetected = true;
	        targetArea = b_w*b_h;
	        centroidX = xC;
	        centroidY = yC;

	        //System.out.println("//PRINTING X, Y//");
	        //System.out.println(centroidX);
	        //System.out.println(centroidY);

	        int destIndex = 0;
	        for (int j = 0; j < arraySize; ++j) {
	          if (connIm[j]-1 == i) {
	            blobIm[destIndex++] = 255;
	          } else {
	            blobIm[destIndex++] = 0;
	          }
	        }
	        /*
	        System.out.println("Centroid coordinates: " + xC + ", " + yC);
	        System.out.println("Area: " + b_w*b_h);
	        System.out.println("");
	        */
	    }
	    break;
	    
      }
      /*
      if (r > 10 && minArea / maxArea > 0.80) {
        return true;
      }
      */
    }
	} // (Solution)

	// (Solution)
	// (Solution)
	/** //(Solution)
	 * <p>Compute range and bearing.</p> //(Solution)
	 * //(Solution)
	 * <p>Note: these are very rough measurments, sufficient for task.</p> //(Solution)
	 * //(Solution)
	 * <p>Using pinhole camera assumption (Horn, Ch2).</p> //(Solution)
	 * //(Solution)
	 * <p>For a point in the world (x,y,z) projected on to image plane IP as //(Solution)
	 * (xp,yp,fp), where fp is the focal length, by simple geometry we know that: //(Solution)
	 * <pre> //(Solution)
	 * xp/fp = x/z, yp/fp=y/z, or fp=xp*z/x //(Solution)
	 * </pre></p> //(Solution)
	 *  //(Solution)
	 * <p>Experimentally, we determine that:<ul> //(Solution)
	 * <li>0.25m = Area of 7200, or radius of 42 pixels</li> //(Solution)
	 * <li>0.5m = Area of  2500,  or radius of 28 pixels</li> //(Solution)
	 * <li>1.0m = Area of  1150,  or radius of 19 pixels</li> //(Solution)
	 * </ul></p> //(Solution)
	 * //(Solution)
	 * <p>We can estimate fp ~= Rpixel*Depth/Rblob, where Rblob ~= .13m , so fp //(Solution)
	 * ~= 107 pixels if we use the 0.5m estimate.</p> //(Solution)
	 * //(Solution)
	 * <p>Now given a area, we can estimate the depth as: Depth = //(Solution)
	 * fp*Rblob/Rpixel.</p> //(Solution)
	 * //(Solution)
	 * <p>We can also estimate the bearing using fp as: //(Solution)
	 * <pre>bearing = atan2(centroidX,fp) //(Solution)
	 * </pre></p> //(Solution)
	 **/ //(Solution)
	public void blobFix() { //(Solution)
    double px = centroidX;
    double py = centroidY;
    double area = targetArea;

    double dForward_cm = -0.3361*py + 57.0385;
    double dLateralLeft_cm = -0.1875*px + 18.4925;

    targetRange = dForward_cm/100.0;
    targetBearing = Math.atan2(dLateralLeft_cm, dForward_cm); 
  /*
		double deltaX = centroidX - width / 2.0; //(Solution)
		targetRange = //(Solution)
			focalPlaneDistance * targetRadius / Math.sqrt(targetArea / Math.PI); //(Solution)
		targetBearing = Math.atan2(deltaX, focalPlaneDistance); //(Solution)
  */
	} //(Solution)

	public double[] getRangeBearing(double cX, double cY, double tA) { //(Solution)
    double px = cX;
    double py = cY;
    double area = tA;

    double dForward_cm = -0.3361*py + 57.0385;
    double dLateralLeft_cm = -0.1875*px + 18.4925;

    double[] results = new double[2];
    results[0] = dForward_cm/100.0; //dist
    results[1] = Math.atan2(dLateralLeft_cm, dForward_cm); //header
    return results;
  /*
		double deltaX = centroidX - width / 2.0; //(Solution)
		targetRange = //(Solution)
			focalPlaneDistance * targetRadius / Math.sqrt(targetArea / Math.PI); //(Solution)
		targetBearing = Math.atan2(deltaX, focalPlaneDistance); //(Solution)
  */
	} //(Solution)


	//(Solution)
	/**  //(Solution)
	 * <p>Compute the translational velocity command using a //(Solution)
	 * P-controller on current translation error.</p> //(Solution)
	 *  //(Solution)
	 * @return translational velocity command //(Solution)
	 */ //(Solution)
	protected void computeTranslationVelocityCommand() { //(Solution)
		double translationError = targetRange - desiredFixationDistance; //(Solution)
		if (Math.abs(translationError) < translationErrorTolerance) //(Solution)
			translationVelocityCommand = 0.0; //(Solution)
		else //(Solution)
			translationVelocityCommand = // (Solution)
				Math.max(-translationVelocityMax, //(Solution)
						Math.min(translationVelocityMax, //(Solution)
								translationError * translationVelocityGain)); //(Solution)
	} //(Solution)

	//(Solution)
	/** //(Solution)
	 * <p>Compute the rotational velocity command using a P-controller //(Solution)
	 * on the current rotation error, {@link #targetBearing}.<\p>  //(Solution)
	 *  //(Solution)
	 * @return rotation velocity command //(Solution)
	 */ //(Solution)
	protected void computeRotationVelocityCommand() { //(Solution)
		double rotationError = targetBearing; //(Solution)
		if (Math.abs(rotationError) < rotationErrorTolerance) //(Solution)
			rotationVelocityCommand = 0.0; //(Solution)
		else //(Solution)
			rotationVelocityCommand = // (Solution)
				Math.max(-rotationVelocityMax, //(Solution)
						Math.min(rotationVelocityMax, //(Solution)
								rotationError * rotationVelocityGain)); //(Solution)
	} //(Solution)

	//(Solution)
	/**
	 * <p>Computes frame rate of vision processing</p>
	 */
	private void stepTiming() {
		double currTime = System.currentTimeMillis();
		stepCounter++;
		// if it's been a second, compute frames-per-second
		if (currTime - lastStepTime > 1000.0) {
			//double fps = (double) stepCounter * 1000.0
			// / (currTime - lastStepTime);
			//System.err.println("FPS: " + fps);
			stepCounter = 0;
			lastStepTime = currTime;
		}
	}

	// (Solution)
	/** //(Solution)
	 * <p>Print average RGB over center 1/4 of image.</p> //(Solution)
	 * //(Solution)
	 * @param source image //(Solution)
	 **/ //(Solution)
	public void averageRGB(Image image) {//(Solution)
		double totalRed = 0; //(Solution)
		double totalGreen = 0; //(Solution)
		double totalBlue = 0; //(Solution)
		int numPixels = 0; //(Solution)
		//(Solution)
		for (int y = 0; y < height; y++) { //(Solution)
			for (int x = 0; x < width; x++) { //(Solution)
				int pix = image.getPixel(x, y); // (Solution)
				// if pixel is in central 1/4 of image, add to total (Solution)
				if (x > 3.0 * width / 8.0 && x < 5.0 * width / 8.0 //(Solution)
						&& y > 3.0 * height / 8.0 && y < 5.0 * height / 8.0) { //(Solution)
					totalRed += Image.pixelRed(pix); //(Solution)
					totalGreen += Image.pixelGreen(pix); //(Solution)
					totalBlue += Image.pixelBlue(pix); //(Solution)
					numPixels++; //(Solution)
				} //(Solution)
			} //(Solution)
		} //(Solution)
		// System.err.println("AverageRGB:  R:" + totalRed/numPixels + " G:" //(Solution)
		//		+ totalGreen/numPixels + " B:" + totalBlue/numPixels); //(Solution)
	} // (Solution)

	/** //(Solution)
	 * <p>Print average HSB over center 1/4 of image.</p> //(Solution)
	 * //(Solution)
	 * @param image source image (as float image) //(Solution)
	 **/ //(Solution)
	protected void averageHSB(Image image) { //(Solution)
		float totalHue = 0; //(Solution)
		float totalSat = 0; //(Solution)
		float totalBri = 0; //(Solution)
		int numPixels = 0; //(Solution)
		//(Solution)
		for (int y = 0; y < height; y++) { //(Solution)
			for (int x = 0; x < width; x++) { //(Solution)
				int pix = image.getPixel(x, y); // (Solution)
				int r = Image.pixelRed(pix); // (Solution)
				int g = Image.pixelGreen(pix); // (Solution)
				int b = Image.pixelBlue(pix); // (Solution)
				float[] hsb = Color.RGBtoHSB(r, g, b, null); // (Solution)
				if (x > 3.0 * width / 8.0 && x < 5.0 * width / 8.0 //(Solution)
						&& y > 3.0 * height / 8.0 && y < 5.0 * height / 8.0) { //(Solution)
					totalHue += hsb[0]; //(Solution)
					totalSat += hsb[1]; //(Solution)
					totalBri += hsb[2]; //(Solution)
					numPixels++; //(Solution)
				} //(Solution)
			} //(Solution)
		} //(Solution)
		System.err.println("AverageHSB: H:" + totalHue/numPixels + " S:" //(Solution)
				+ totalSat/numPixels + " B:" + totalBri/numPixels); //(Solution)
	} //(Solution)

	/** // (Solution)
	 * <p>Highlight the blob in the given image, while setting all // (Solution)
	 * background pixels to grayscale<\p> // (Solution)
	 * // (Solution)
	 * @param src source image // (Solution)
	 * @param dest destination image // (Solution)
	 */ // (Solution)
	protected void markBlob(Image src, Image dest) { // (Solution)
		int maskIndex = 0; //(Solution)
		//(Solution)
		for (int y = 0; y < height; y++) { //(Solution)
			for (int x = 0; x < width; x++) { //(Solution)
				if (targetDetected && blobMask[maskIndex++] > 0) { //(Solution)
					dest.setPixel(x, y, (byte) 0,(byte) 0xff,(byte) 0); // (Solution)
				} else { //(Solution)
					int pix = src.getPixel(x, y); //(Solution)
					//(Solution)
					byte av = (byte) (((Image.pixelRed(pix) & 0xff) +
							(Image.pixelGreen(pix) & 0xff) +
							(Image.pixelBlue(pix) & 0xff)) / 3); //(Solution)
					dest.setPixel(x, y, av, av, av);//(Solution)
				} //(Solution)
			} //(Solution)
		} //(Solution)
	} // (Solution)

	//(Solution)

	/** //(Solution)
	 * <p>BallPixel pixel-level classifier. Threshold HSB image //(Solution)
	 * based on hue distance to target hue and saturation level, //(Solution)
	 * and build binary mask.</p> //(Solution)
	 *  //(Solution)
	 * @param src source image (float) //(Solution)
	 * @param src dest image (int) //(Solution)
	 **/ //(Solution)
	protected void blobPixel(Image src, int[] mask, double targetHueLevel) { //(Solution)
		//(Solution)
		int maskIndex = 0; //(Solution)
		//(Solution)
		//// use to accumulate for avg hue and saturation (Solution)
		//double avg_h = 0.0; // (Solution)
		//double avg_s = 0.0; // (Solution)
		// (Solution)
        int cnt = 0;
		for (int y = 0; y < height; y++) { // (Solution)
			for (int x = 0; x < width; x++) { // (Solution)
				int pix = src.getPixel(x, y); // (Solution)
				float[] hsb = Color.RGBtoHSB(Image.pixelRed(pix)&0xFF, // (Solution)
						Image.pixelGreen(pix)&0xFF, // (Solution)
						Image.pixelBlue(pix)&0xFF, null); // (Solution)
				// (Solution)
				//avg_h += pix.getHue(); // (Solution)
				//avg_s += pix.getSaturation(); // (Solution)
				// (Solution)
				double hdist = hsb[0] - targetHueLevel; // (Solution)

				if (targetHueLevel == 0.0){
					if (hdist < 0) hdist *= -1; // (Solution)
					// handle colorspace wraparound (Solution)
					if (hdist > 0.5) { // (Solution)
						hdist = 1.0 - hdist; // (Solution)
					} 
				}

				// (Solution)
				// (Solution)
				// classify pixel based on saturation level (Solution)
				// and hue distance (Solution)
				if (hsb[1] > saturationLevel && hdist < hueThreshold && hsb[2] > 0.3) { // (Solution)
         			++cnt;
					mask[maskIndex++] = 255; // (Solution)
				} else { // (Solution)
					mask[maskIndex++] = 0; // (Solution)
				} // (Solution)	
			} // (Solution)
		} // (Solution)
    //System.out.println("There are " + cnt + " red pixels. Width="+width + " height=" + height);
		// (Solution)
		// avg_h /= width * height; // (Solution)
		// avg_s /= width * height; // (Solution)
		// System.err.println("Total Avgerage Hue, Sat: "+avg_h+" "+avg_s); // (Solution)
	} // (Solution)


	public boolean isDouble(int width, int height){
		float ratio1 = (float) (width/height);
		float ratio2 = (float) (height/width);

		if (ratio1>rectThreshold || ratio2>rectThreshold){
			return true;
		}
		return false;

	}
	/**
	 * <p>Segment out a blob from the src image (if a good candidate exists).</p>
	 *
	 * <p><code>dest</code> is a packed RGB image for a java image drawing
	 * routine. If it's not null, the blob is highlighted.</p>
	 *
	 * @param src the source RGB image, not packed
	 * @param dest the destination RGB image, packed, may be null
	 */
	public void apply(Image src, Image dest) {

		stepTiming(); // monitors the frame rate

		// Begin Student Code

		//averageRGB(src); // (Solution)
		//averageHSB(src); // (Solution)

		if (useGaussianBlur) {// (Solution)
			byte[] srcArray = src.toArray();// (Solution)
			byte[] destArray = new byte[srcArray.length]; // (Solution)
			if (approximateGaussian) { // (Solution)
				GaussianBlur.applyBox(srcArray, destArray, src.getWidth(), src.getHeight());
			} // (Solution)
			else { // (Solution)
				GaussianBlur.apply(srcArray, destArray, width, height); // (Solution)
			} // (Solution)
			src = new Image(destArray, src.getWidth(), src.getHeight()); // (Solution)
		}
		blobPixel(src, blobPixelRedMask, targetRedHueLevel); //(Solution)
		blobPixel(src, blobPixelBlueMask, targetBlueHueLevel); //(Solution)
		blobPixel(src, blobPixelYellowMask, targetYellowHueLevel); //(Solution)
		blobPixel(src, blobPixelGreenMask, targetGreenHueLevel); //(Solution)
		max_area = -1;


		blobPresent(blobPixelRedMask, imageConnected, blobMask);
		blobPresent(blobPixelBlueMask, imageConnected, blobMask);
		blobPresent(blobPixelYellowMask, imageConnected, blobMask);
		blobPresent(blobPixelGreenMask, imageConnected, blobMask);
		 //(Solution)
		if (targetDetected) { // (Solution)
      System.out.println("Target detected!");
			blobFix(); // (Solution)
			computeTranslationVelocityCommand(); // (Solution)
			computeRotationVelocityCommand(); // (Solution)
			// System.err.println("Bearing (Deg): " + (targetBearing*180.0/Math.PI)); // (Solution)
			// System.err.println("Range (M): " + targetRange); // (Solution)
		} else { // (Solution)
			// System.err.println("no target"); // (Solution)
			translationVelocityCommand = 0.0; // (Solution)
			rotationVelocityCommand = 0.0; // (Solution)
		} // (Solution)
		// (Solution)
		// System.err.println("Tracking Velocity: " + // (Solution)
		//		translationVelocityCommand + "m/s, " + // (Solution)
		//		rotationVelocityCommand + "rad/s"); // (Solution)
		// For a start, just copy src to dest. // (Solution)
		//if (dest != null) { // (Solution)
			// (Solution)
			//Histogram.getHistogram(src, dest, true); // (Solution)
			markBlob(src, dest); // (Solution) TODO
			// (Solution)
		//} // (Solution)
		// End Student Code
	}

	public boolean apply_background(Image src, Image dest) {

		stepTiming(); // monitors the frame rate

		// Begin Student Code

		//averageRGB(src); // (Solution)
		//averageHSB(src); // (Solution)

		if (useGaussianBlur) {// (Solution)
			byte[] srcArray = src.toArray();// (Solution)
			byte[] destArray = new byte[srcArray.length]; // (Solution)
			if (approximateGaussian) { // (Solution)
				GaussianBlur.applyBox(srcArray, destArray, src.getWidth(), src.getHeight());
			} // (Solution)
			else { // (Solution)
				GaussianBlur.apply(srcArray, destArray, width, height); // (Solution)
			} // (Solution)
			src = new Image(destArray, src.getWidth(), src.getHeight()); // (Solution)
		}
		blobPixel(src, blobPixelRedMask, targetRedHueLevel); //(Solution)
		blobPixel(src, blobPixelBlueMask, targetBlueHueLevel); //(Solution)
		blobPixel(src, blobPixelYellowMask, targetYellowHueLevel); //(Solution)
		blobPixel(src, blobPixelGreenMask, targetGreenHueLevel); //(Solution)
		max_area = -1;


		blobPresent(blobPixelRedMask, imageConnected, blobMask);
		blobPresent(blobPixelBlueMask, imageConnected, blobMask);
		blobPresent(blobPixelYellowMask, imageConnected, blobMask);
		blobPresent(blobPixelGreenMask, imageConnected, blobMask);
		 //(Solution)
		if (targetDetected) { // (Solution)
	    	return true;
			// System.err.println("Bearing (Deg): " + (targetBearing*180.0/Math.PI)); // (Solution)
			// System.err.println("Range (M): " + targetRange); // (Solution)
		} else { // (Solution)
			// System.err.println("no target"); // (Solution)
			return false;
		} // (Solution)
		// (Solution)
		// System.err.println("Tracking Velocity: " + // (Solution)
		//		translationVelocityCommand + "m/s, " + // (Solution)
		//		rotationVelocityCommand + "rad/s"); // (Solution)
		// For a start, just copy src to dest. // (Solution)
		//if (dest != null) { // (Solution)
			// (Solution)
			//Histogram.getHistogram(src, dest, true); // (Solution)
			//markBlob(src, dest); // (Solution) TODO
			// (Solution)
		//} // (Solution)
		// End Student Code
	}
}
