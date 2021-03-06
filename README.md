# Gate_Detection_Viola_Jones
A gate detection algorithm based on cascade object detector for drone race.  

## Objectives
The algorithm is able to perform real time rectangle gate detection by extracting the vertex. Four base pattern detectors are trained in advanced and used to detect the vertex.  

## Result
The red bbox is the detection of top left; yellow for top right; blue for bottom right and green for bottom left. The pink polygon is the detection of the gate.  

<div align=center><img width="640" height="480" src="https://github.com/0Jiahao/Gate_Detection_Viola_Jones/blob/master/result/result.gif"/></div>  

## Future work
The detector does not have the property rotation-invariance, raw image should be rotated in advanced (data gathered from agent);  
Only darker corners' detectors are implemented in the current version. Lighter corner detectors can improve its performance in various enviroments.
