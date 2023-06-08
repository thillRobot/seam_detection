# notes - June 08, 2023
# Tristan Hill

## Overview

The big picture goal is to develop a product that uses 3D sensing on a robot arm end effector to automate a welding operation. 

## Sensors

### Mech Mind

	Solutions > Assembling, Locating, and Gluing
		"Mech-Mind 3D vision system detects and locates parts with high accuracy even in long-range working distance. "

		Specs: 
		Objects
	
		Metal parts of different shapes (sheets, blocks, columns, rings, etc.) and types (aluminum, iron, copper, etc.)

		Minimum Object Size
			
		Approx. 20mm x 8.4mm x 5mm (e.g. M5 hexagon socket screw)


		  	> 3D Vision Guided Automatic Trajectory Generation with Mech-Mind (video)

		  	 >> claims calibration accuracy of 1mm @ 3m


	Products > Mech Eye (Camera) 

		Model: Nano

			dimensions: 145mmx51mmx85mm
			weight:     0.7kg	
			communication: ethernet
			sensor: CMOS
			  	 	
			> Mech Vision (Software)	

				> Various Typical Vision Algorithms - 3D model creating and matching, 2D matching, special algorithms for trajectory and measurement


			> Mech Viz (Robot Software)	
			> Mech DLK (Deep Learning)


	Support  > Software (https://docs.mech-mind.net/latest/en-GB/MechEye/MechEye.html) (updated 2023)

	Mech Eye SDK for Windows SDK and Ubuntu (AMD64 or ARM)

	API for C++, C#, and Python		

	"ROS API in available on Github"


	Initial thoughts: no red flag, support looks up to date and reasonable thorough	

### Basler

	#### Blaze (sony depthsense imx556)

	technology: Time of Flight (pulsed time of flight principle)
	sensor: Sony DepthSense and onboard depth image processing
	data: 3D point cloud, intensity image, range map, and confidence map

	resolution: +/- 5mm, 30 fps


	#### Stereo Camera 

	nvidia tegra GPU
	resolution: model 65        
	            0.04mm @ 0.2m  
	            0.9mm  @ 1.0m

	            model 160 
	            0.06 mm @ 0.5 m
				0.3 mm @ 1.0 m
				1.0 mm @ 2.0 m
				2.2 mm @ 3.0 m

 	communication: GigE

 	size: model 65, 135 x 75 x 96 mm 
 		  model 160, 230 x 75 x 84 mm 

 	#### RGBD (ACE single Camera + Blaze depth sensor)	  


### Solomon

	#### Solscan (3d Camera)- Deploying SolScan opens up a wide range of possibilities, including object recognition and classification, precise measurements, bin picking, robot guiding, accurate object scanning, and 3D modeling for virtual reality and augmented reality.





Note: look in to GenICam, (Generic Interface for Cameras) - covers GigE Vision, USB3 Vision, Camera Link, IEEE 1394, and various camera types and image formats 

	a generic programming interface for machine vision cameras (industrial)