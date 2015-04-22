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
 * Histogram functions for generating, normalizing, and overlaying
 * a histogram in an image.
 * 
 * @author unknown
 * @author Carrick Detweiler
 */
public class Histogram {

	/**
	 * <p>Gets an image with a histogram of source overlayed.  If
	 * destination is null a copy of source is allocated, otherwise the
	 * histogram is overlayed on top of destination.  It is fine (and
	 * desirable for speed) if source == destination.</p>
	 *
	 * <p>Note, source and destination must have the same width and height.</p>
	 *
	 * @param source image to compute the histogram from
	 * @param destination if non-null histogram is written on top of this
	 * @param hsbHistogram produces a HSB histogram if true, else RGB histogram
	 * @return desitnation or a new image if destination is null
	 **/
	public static Image getHistogram(Image source, Image destination,
			boolean hsbHistogram){
		//First check if we need to allocate the destination
		if(destination == null){
			destination = new Image(source);
		}

		//Verify source and destination have the same size
		if(source.getWidth() != destination.getWidth()
				|| source.getHeight() != destination.getHeight()){
			System.err.println(
			"Error: Histogram.getHistogram() passed source and destination with different sizes");
			System.err.println("    source width: " +source.getWidth()
					+ " height: " + source.getHeight());
			System.err.println("    destination width: " +destination.getWidth()
					+ " height: " + destination.getHeight());
			return destination;
		}

		//Now tally all the pixel values
		float histogram[][] = makeHistogram(source,hsbHistogram);

		//Normalize the histogram to range between 0.0 and 1.0
		normalizeHistogram(histogram);

		//Draw the histogram on the output image
		overlayHistogram(destination,histogram);

		return destination;
	}

	/**
	 * <p>Overlay the histogram onto destination.  See {@link
	 * makeHistogram()} and {@link normalizeHistogram()} for the
	 * histogram format.  This method expects that the histogram is
	 * normalized between 0.0 and 1.0.</p>
	 *
	 * @param destination image to overlay the histogram on
	 * @param histogram the histogram to put on destination.
	 **/
	public static void overlayHistogram(Image destination, float histogram[][]) {
		int height = destination.getHeight();

		// The fraction of the height to use as the max histogram value
		double desiredHeightFraction = 0.4;

		// The maximum height
		int histImgHeight = (int)(height * desiredHeightFraction);

		// Begin Student Extra Code

		// The histogram code provided is somewhat basic and can be
		// improved upon.  For instance you could blend the channels to
		// create a photoshop-like histogram.

		// End Student Extra Code

		int numBlue, numGreen, numRed;
		for (int w=0; w < histogram.length; w++) {
			// Get the correct count
			numRed = Math.round(histogram[w][0] * histImgHeight);
			numGreen = Math.round(histogram[w][1] * histImgHeight);
			numBlue = Math.round(histogram[w][2] * histImgHeight);

			int h = height-1;

			// Now stack the colors
			for (;numRed > 0; numRed--) {
				destination.setPixel(w,h--,
						(byte) 0xff, (byte) 0, (byte) 0);
			}
			for (;numGreen > 0; numGreen--) {
				destination.setPixel(w,h--,
						(byte) 0, (byte) 0xff, (byte) 0);
			}
			for (;numBlue > 0; numBlue--) {
				destination.setPixel(w,h--,
						(byte) 0, (byte) 0, (byte) 0xff);
			}
		}
	}

	/**
	 * <p>Creates a histogram from an image. Returned is an array with
	 * min(image width, 256) elements and 3 channels (eg an hist[256][3]).</p>
	 *
	 * <p>In an rgb histogram hist[][0] is red, hist[][1] is green,
	 * hist[][2] blue.  In an hsb histogram hist[][0] is hue, hist[][1]
	 * is saturation, hist[][2] brightness.  In an image with width >= 256
	 * a particular bin, eg hist[42][2], counts the number of pixels in
	 * the image which have a blue/brightness value of 42.  In smaller
	 * images these are scaled appropriately.</p>
	 *
	 * @param source image to compute the histogram from
	 * @param hsbHistogram produces a HSB histogram if true, else RGB histogram
	 * @return 2D histogram array
	 **/
	public static float[][] makeHistogram(Image source, boolean hsbHistogram) {
		int width = source.getWidth();
		int height = source.getHeight();

		// Scale the number of histogram boxes if our image is smaller than
		// 256 pixels wide.
		double scale;
		if(width < 256) {
			scale = (double)256 / width;
		}
		else {
			// just limit it to 256 bins
			scale = 1;
		}

		// Allocate the bins, 3 channels for rgb or hsb
		float histogram[][] = new float[Math.min(width, 256)][3];

		// Begin Student Code

    int w = Math.min(width, 256);
    int binSize = 256 / w;
    for (int i =0; i < source.getWidth(); i++)
    {
      for(int j=0; j < source.getHeight(); j++){
        int pixel = source.getPixel(i,j);
	int pRed = Image.pixelRed(pixel) & 0xFF;
        int pBlue = Image.pixelBlue(pixel) & 0xFF;
        int pGreen = Image.pixelGreen(pixel) & 0xFF;
        if(!hsbHistogram){
        histogram[pRed*w/256][0]++;
        histogram[pGreen*w/256][1]++;
        histogram[pBlue*w/256][2]++;
      } 
	else {
        float[] hsb = Color.RGBtoHSB(pRed,
				     pGreen,
				     pBlue,
				     null);
	histogram[(int) (Math.min(hsb[0]*w, w-1))][0]++;
        histogram[(int) (Math.min(hsb[1]*w, w-1))][1]++;
        histogram[(int) (Math.min(hsb[2]*w, w-1))][2]++;
	}
      }
    }

		
		// End Student Code

		return histogram;
	}

	/**
	 * <p>Normalizes the 2D histograms by the maximum sum of
	 * all histogram channels in a given bin.  i.e. find the
	 * sum of r+g+b for all bins, and normalize based on the
	 * maximum, which will have value 1.0.  This is used for
	 * stacking the histograms during overlay.<\p>
	 *
	 * @param 2D histogram array
	 */
	private static void normalizeHistogram(float[][] histogram) {
		if(histogram == null)
			return;

		int numBins = histogram.length;
		int numChannels = histogram[0].length;

		// find maximum column total (ie. r+g+b)
		float maxColTotal = 0;
		float colTotal;
		for (int w = 0; w < numBins; w++) {
			colTotal = 0;
			for (int c = 0; c < numChannels; c++) {
				colTotal += histogram[w][c];
			}
			if (colTotal > maxColTotal) {
				maxColTotal = colTotal;
			}
		}

		// normalize histogram based on the maximum column total
		for (int w = 0; w < numBins; w++) {
			for (int c = 0; c < numChannels; c++) {
				histogram[w][c] /= maxColTotal;
			}
		}
	}
}
