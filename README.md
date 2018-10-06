# Gate_Detection_Viola_Jones
An gate detection algorithm based on cascade object detector for drone race.
## Objectives
The algorithm is able to perform rectangle gate detection by extracting the vertex. Four base pattern detectors are trained in advanced and used to detect the vertex.
## Result
The red bbox is the detection of top left; yellow for top right; green for bottom right and blue for bottom left. The pink polygon is the detection of the gate.  
<div align=center><src="https://github.com/0Jiahao/Gate_Detection_Viola_Jones/blob/master/result/result.gif"/></div>
## Further work
The detector does not have the property rotation-invariance, raw image should be rotated in advanced (data gathered from agent)
