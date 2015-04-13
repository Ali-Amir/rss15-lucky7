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

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

/**
 * <p>Class for testing the histogram without the robot</p>
 *
 * @author Carrick Detweiler
 **/
public class HistogramTest{


	/**
	 * <p>Gets an rss VisualServo.Image type of image from the given
	 * BufferedImage.</p>
	 *
	 * @param image BufferedImage to convert to a VisualServo.Image
	 * @return VisualServo.Image representation of image
	 **/
	public static Image getRSSImage(BufferedImage image){
		int width = image.getWidth();
		int height = image.getHeight();
		Image rssimg = new Image(width,height);

		System.out.println("BufferedImage.getRSSImage input width: " + width + " height: " + height);

		//Copy the data in
		for(int i = 0; i < width; i++){
			for(int j = 0; j < height; j++){
				int val = image.getRGB(i,j);
				rssimg.setPixel(i,j,val);
			}
		}

		return rssimg;
	}


	/**
	 * <p>Gets a BufferedImage from an rss VisualServo.Image</p>
	 *
	 * @param image rss image
	 * @return a BufferedImage representation of image
	 **/
	public static BufferedImage getBufferedImage(Image image){
		int width = image.getWidth();
		int height = image.getHeight();
		BufferedImage bimage = new BufferedImage(width,height,BufferedImage.TYPE_INT_RGB);

		System.out.println("BufferedImage.getBufferedImage input width: " + width + " height: " + height);

		//Copy the data in
		for(int i = 0; i < width; i++){
			for(int j = 0; j < height; j++){
				int p = image.getPixel(i,j);
				bimage.setRGB(i,j,p);
			}
		}

		return bimage;
	}

	/**
	 * <p>Just read in the image passed image and the desired output image</p>
	 **/
	public static void main(String[] arg) {
		if(arg.length != 2){
			System.out.println("Usage: java VisualServo.HistogramTest inimage outimage");
			System.exit(1);
		}

		String inputFilename = arg[0];
		String outputFilename = arg[1];
		System.out.println("Reading image " + inputFilename
				+ " writting image " + outputFilename);

		try{
			//Get the image in
			BufferedImage inputImage = ImageIO.read(new File(inputFilename));
			Image rssImage = HistogramTest.getRSSImage(inputImage);

			Histogram.getHistogram(rssImage,rssImage,false);

			//Now ouput it
			BufferedImage outputImage = HistogramTest.getBufferedImage(rssImage);
			ImageIO.write(outputImage,"JPEG",new File(outputFilename));
		}
		catch (IOException e){
			e.printStackTrace();
		}
	}

}
