# RANSAC-KD_Tree
* Implementation of RANSAC(Random Sampling and Consensus) algorithm to detect lines from the given set of points given to us.
* We use KD Tree data structure for code optimisation

### File Structure ->
* kd_data.txt - 
Text file containing the x coordinate and y coordinates of 65 points. These 65 points lie on or close(because of the noise in the data) to the 3 lines, "x+y=10", "x=8" and "x=y"

* without_KD_tree.cpp - 
Reading the text file, "kd_data.txt" for the 65 data points.
Program is using RANSAC to randomly select 2 points to hypothesize a line and then K-means clustering to detect three lines, "x+y=10", "x=8" and "x=y" 

* using_KD_tree.cpp -
Reading the text file, "kd_data.txt" which has 65 data points.
Program is using the KD tree data structure instead of the brute force approach in RANSAC. Instead of checking for all the points, whether they lie within a certain threshold distance of the line or not, we only check for the K-nearest neighbours of the point.
