/*
 * Copyright (C) 2014 Ali-Amir Aldan.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.rosjava.challenge.vision;

import java.awt.Color;
import com.github.rosjava.challenge.gui.Image;

/**
 * BlobTracking performs image processing and tracking for the Vision 
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

	protected float histogram[][]; //(Solution)
	// (Solution)
	protected int blobMask[] = null; //(Solution)
	protected int blobPixelMask[] = null; //(Solution)
	protected int imageFiltered[] = null; //(Solution)
	protected int imageConnected[] = null; //(Solution)
	protected float imageHsb[] = null; //(Solution)
	// (Solution)
	public double targetHueLevel; // (Solution)
	public double targetRadius; // (Solution)
	public double hueThreshold; // (Solution)
	public double saturationLevel; // (Solution)
	public double blobSizeThreshold; // (Solution)
	public double desiredFixationDistance; // (Solution)
	public double translationErrorTolerance; // (Solution)
	public double rotationErrorTolerance; // (Solution)
	public boolean useGaussianBlur; // (Solution)
	public boolean approximateGaussian; // (Solution)
	public double translationVelocityGain; // (Solution)
	public double translationVelocityMax; // (Solution)
	public double rotationVelocityGain; // (Solution)
	public double rotationVelocityMax; // (Solution)
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
		blobPixelMask = new int[width * height]; // (Solution)
		imageConnected = new int[width * height]; // (Solution)
		imageHsb = new float[width * height * 3]; // (Solution)
	}

	//(Solution)
	//(Solution)
	/** // (Solution)
	 * <p>Apply connected components analysis to pick out the largest blob. Then // (Solution)
	 * build stats on this blob.</p> // (Solution)
	 **/ // (Solution)
	protected void blobPresent(int[] threshIm, int[] connIm, int[] blobIm) { // (Solution)
		// (Solution)
		connComp.doLabel(threshIm, connIm, width, height); // (Solution)
		// (Solution)
		int colorMax = connComp.getColorMax(); // (Solution)
		int countMax = connComp.getCountMax(); // (Solution)
		// System.out.println("Count max -- num pixels is :  " + countMax);// (Solution)
		// System.out.println("Fraction of num pixels is  " + ((float)countMax) / (width*height));// (Solution)
		// (Solution)
		if (countMax > blobSizeThreshold * height * width) { // (Solution)
			int sx = 0; // (Solution)
			int sy = 0; // (Solution)
			targetArea = countMax; // (Solution)
			int destIndex = 0; // (Solution)
			for (int y = 0; y < height; y++) { // (Solution)
				for (int x = 0; x < width; x++) { // (Solution)
					if (connIm[destIndex] == colorMax) { // (Solution)
						sx += x; // (Solution)
						sy += y; // (Solution)
						blobIm[destIndex++] = 255; // (Solution)
					} else { // (Solution)
						blobIm[destIndex++] = 0; // (Solution)
					} // (Solution)
				} // (Solution)
			} // (Solution)
			centroidX = sx / (double) countMax; // (Solution)
			centroidY = sy / (double) countMax; // (Solution)
			targetDetected = true; // (Solution)
		} // (Solution)
		else { // (Solution)
			targetDetected = false; // (Solution)
		} // (Solution)
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
		double deltaX = centroidX - width / 2.0; //(Solution)
		targetRange = //(Solution)
			focalPlaneDistance * targetRadius / Math.sqrt(targetArea / Math.PI); //(Solution)
		targetBearing = Math.atan2(deltaX, focalPlaneDistance); //(Solution)
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
								-rotationError * rotationVelocityGain)); //(Solution)
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
				int r = Image.pixelRed(pix)&0xFF; // (Solution)
				int g = Image.pixelGreen(pix)&0xFF; // (Solution)
				int b = Image.pixelBlue(pix)&0xFF; // (Solution)
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
	protected void blobPixel(Image src, int[] mask) { //(Solution)
		//(Solution)
		int maskIndex = 0; //(Solution)
		//(Solution)
		//// use to accumulate for avg hue and saturation (Solution)
		//double avg_h = 0.0; // (Solution)
		//double avg_s = 0.0; // (Solution)
		// (Solution)
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
				if (hdist < 0) hdist *= -1; // (Solution)
				// handle colorspace wraparound (Solution)
				if (hdist > 0.5) { // (Solution)
					hdist = 1.0 - hdist; // (Solution)
				} // (Solution)
				// (Solution)
				// classify pixel based on saturation level (Solution)
				// and hue distance (Solution)
				if (hsb[1] > saturationLevel && hdist < hueThreshold) { // (Solution)
					mask[maskIndex++] = 255; // (Solution)
				} else { // (Solution)
					mask[maskIndex++] = 0; // (Solution)
				} // (Solution)
			} // (Solution)
		} // (Solution)
		// (Solution)
		// avg_h /= width * height; // (Solution)
		// avg_s /= width * height; // (Solution)
		// System.err.println("Total Avgerage Hue, Sat: "+avg_h+" "+avg_s); // (Solution)
	} // (Solution)

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
		blobPixel(src, blobPixelMask); //(Solution)
		blobPresent(blobPixelMask, imageConnected, blobMask); //(Solution)
		if (targetDetected) { // (Solution)
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
		if (dest != null) { // (Solution)
			// (Solution)
			//Histogram.getHistogram(src, dest, true); // (Solution)
			markBlob(src, dest); // (Solution)
			// (Solution)
		} // (Solution)
		// End Student Code
	}
}
