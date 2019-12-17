//kd_data.txt contains the x-coordinate followed by the y-coordinate of the points
//We are not using the KD_tree DS here.....we are using the normal RANSAC algo, followed by clustering
//Taking input "kd_data.txt" which has 65 data points
#include<bits/stdc++.h> 
#include <iostream>
#include <fstream>//inlcude the fstream class
#include <math.h> 
#include <vector>
#include <stdlib.h>
#include <stdio.h>
using namespace std; 

struct Point
{
	double x;
	double y;
};

struct line_datatype
{
    double d;
    double theeta;
};

struct dataset
{
    line_datatype line;
    int cluster_no;
};

double dist(Point Pt, line_datatype line)
{
    double dist = abs(Pt.x*cos(line.theeta) + Pt.y*sin(line.theeta) - line.d);
    return dist;
}

double kmeans_dist(line_datatype line1, line_datatype cluster_centre)
{
    double kmeans_d;

    //cout<<"Line(with respect to which the distances of the cluster centres are being calculated): ("<<line1.d<<","<<line1.theeta<<")  "<<"cluster_centre: ("<<cluster_centre.d<<","<<cluster_centre.theeta<<")"<<endl;
    
    kmeans_d = sqrt(pow((line1.d - cluster_centre.d),2) + 10*pow((line1.theeta - cluster_centre.theeta),2));
    return kmeans_d;
    
}


// A structure to represent node of kd tree 
struct Node 
{ 
    Point point; // To store k dimensional point(2 dimensional in our case)
    Node *left, *right; 
}; 
  
// A method to create a node of K D tree 
struct Node* newNode(Point point1) 
{ 
    struct Node* temp = new Node; 
    temp->point = point1; 
    temp->left = temp->right = NULL; 
    return temp; 
} 
  

// Driver program to test above functions 
int main() 
{ 
    struct Node *root = NULL; 
    
	ifstream inFile;//inFile is an object of the ifstream class 
	inFile.open("kd_data.txt");

	//check for Error
	if(inFile.fail())
	{
		cerr<<"Error opening file"<<endl;
		exit(1);
	}

	vector<Point> Pts;
	Point temp_point;
	bool flag = 0;

	while(!inFile.eof())
	{
		if(flag == 0)
		{
			inFile>>temp_point.x;//the >> operator moves us to the next line
			flag = 1;
		}

		else if(flag == 1)
		{
			inFile>>temp_point.y;//the >> operator moves us to the next line
			flag = 0;			
			Pts.push_back(temp_point);
		}
	}

    Point points;//take input from kd_data.txt
  
    int no_points = Pts.size();
  
    vector<line_datatype> detected_lines;
    vector<dataset> lines_to_be_clustered;
    dataset data;

    int index, counter, count, RANSAC_iterations, threshold_for_line, k_means_iteration, no_cluster1, no_cluster2, no_cluster3, k_means_index, no_detected_lines;
    double theeta, d, dist_threshold, dist1, dist2, dist3;//Point normal form of a line: "d = x*cos(theeta) + y*sin(theeta)" has been used
    Point pt1, pt2;//we will randomly select 2 points for RANSAC algo 'iterations' number of times
    line_datatype line;

    srand(time(0));
    no_points = Pts.size();
    k_means_iteration = 1000;//we use 30 iterations of the K-means algo
    RANSAC_iterations = 1000;//the number of times we want to randomly test for a line...ideally we want a big number like 100 or above, but becoming computationally expensive 
    threshold_for_line = 8;//minimum number of points to lie on a line, for the line to be classified as a valid line
    dist_threshold = 0.1;
    no_cluster1 = no_cluster2 = no_cluster3 = 0;

    cout<<no_points<<" Points found in the data set provided"<<endl;

    int i,j;
    int rand1, rand2, rand3;
    
    for(count=0;count<RANSAC_iterations;count++)
    {
        //cout<<"Iteration: "<<count<<endl;
        rand1 = rand()%no_points;
        rand2 = rand()%no_points;

        if(rand1 == rand2)
            rand2 = rand1+1;

        pt1 = Pts[rand1];
        pt2 = Pts[rand2];

        if(abs(pt2.y-pt1.y) < 0.0001)
        {
            if((pt1.x-pt2.x > 0 && pt2.y-pt1.y > 0) || (pt1.x-pt2.x < 0 && pt2.y-pt1.y < 0))
                line.theeta = 1.5705;
            else
                line.theeta = -1.5705;
        }
        
        else
            line.theeta = atan( ((double)(pt1.x-pt2.x)) / ((double)(pt2.y-pt1.y)) );
        
        line.d = (pt1.x)*(cos(line.theeta)) + (pt1.y)*(sin(line.theeta));

        counter = 0;
        for(index=0;index<no_points;index++)
        {
            if(dist(Pts[index],line) < dist_threshold)
                counter++;
        }

        if(counter>threshold_for_line)
        {
            detected_lines.push_back(line);
            cout<<endl;
            cout<<"Detected Line number: "<<detected_lines.size()<<endl;
            cout<<"("<<pt1.x<<","<<pt1.y<<")  "<<"("<<pt2.x<<","<<pt2.y<<")  "<<counter<<endl;
            cout<<"Line: ";
            cout<<cos(line.theeta)<<"x + "<<sin(line.theeta)<<"y"<<"="<<line.d<<endl;
        }   

    }

    no_detected_lines = detected_lines.size();
    cout<<"The number of lines detected are: "<<no_detected_lines<<endl<<endl;

//The lines have been detected perfectly
//Now we will cluster the lines into groups of 3(Assumed we know the number of lines beforehand)
    cout<<"The detected_lines are: "<<endl<<endl;
    
    ////////////Now comes the code for clustering of the lines//////////
    ////Assumption: There are 3 lines being formed from the data set////
    
    rand1 = rand()%(no_detected_lines-1);
    rand2 = rand()%(no_detected_lines-1);
    rand3 = rand()%(no_detected_lines-1);

    //So that rand1,2,3 are all different so that 2 selected cluster centres are not the same
    while(rand1 == rand2 || rand1 == rand3 || rand2 == rand3)
    {
        rand1 = rand()%(no_detected_lines-1);
        rand2 = rand()%(no_detected_lines-1);
        rand3 = rand()%(no_detected_lines-1);
    }

    cout<<"rand1: "<<rand1<<endl;
    cout<<"rand2: "<<rand2<<endl;
    cout<<"rand3: "<<rand3<<endl;

    line_datatype cluster_centre1 = detected_lines[rand1];
    line_datatype cluster_centre2 = detected_lines[rand2];
    line_datatype cluster_centre3 = detected_lines[rand3];

    line_datatype temp_cluster_centre1;
    line_datatype temp_cluster_centre2;
    line_datatype temp_cluster_centre3;
    
    for(count=0;count<no_detected_lines;count++)
    {
        cout<<"count: "<<count<<endl;
        data.line = detected_lines[count];
        //data is an object of the data type "data_type", which contains both the line details and the cluster number details

        cout<<"detected_lines["<<count<<"]"<<"("<<detected_lines[count].d<<","<<detected_lines[count].theeta<<")"<<endl;
        cout<<"cluster_centre1: ("<<cluster_centre1.d<<","<<cluster_centre1.theeta<<")"<<endl;
        cout<<"cluster_centre2: ("<<cluster_centre2.d<<","<<cluster_centre2.theeta<<")"<<endl;
        cout<<"cluster_centre3: ("<<cluster_centre3.d<<","<<cluster_centre3.theeta<<")"<<endl;
        //cout<<"dist1: ";
        dist1 = kmeans_dist(detected_lines[count], cluster_centre1);
        //cout<<"dist2: ";
        dist2 = kmeans_dist(detected_lines[count], cluster_centre2);
        //cout<<"dist3: ";
        dist3 = kmeans_dist(detected_lines[count], cluster_centre3);
        
        cout<<"dist1: "<<dist1<<endl;
        cout<<"dist2: "<<dist2<<endl;
        cout<<"dist3: "<<dist3<<endl;

        if(dist1<=dist2 && dist1<=dist3)
        {
            data.cluster_no = 1;
            cout<<"Cluster no: 1"<<endl<<endl;
        }

        else if(dist2<=dist1 && dist2<=dist3)
        {
            data.cluster_no = 2;
            cout<<"Cluster no: 2"<<endl<<endl;
        }

        else
        {
            data.cluster_no = 3;
            cout<<"Cluster no: 3"<<endl<<endl;
        }
        
        lines_to_be_clustered.push_back(data);
    }

    for(k_means_index = 1; k_means_index < k_means_iteration; k_means_index++)
    {
        cout<<"k-means iteration number: "<<k_means_index<<endl;
        no_cluster1 = no_cluster2 = no_cluster3 = 0;
        
        temp_cluster_centre1.d = temp_cluster_centre1.theeta = 0;
        temp_cluster_centre2.d = temp_cluster_centre2.theeta = 0;
        temp_cluster_centre3.d = temp_cluster_centre3.theeta = 0;

        //Assigning cluster numbers to all the detected lines
        for(count=0;count<no_detected_lines;count++)
        {
            cout<<"count: "<<count<<endl;

            cout<<"detected_lines["<<count<<"]"<<"("<<detected_lines[count].d<<","<<detected_lines[count].theeta<<")"<<endl;
            cout<<"cluster_centre1: ("<<cluster_centre1.d<<","<<cluster_centre1.theeta<<")"<<endl;
            cout<<"cluster_centre2: ("<<cluster_centre2.d<<","<<cluster_centre2.theeta<<")"<<endl;
            cout<<"cluster_centre3: ("<<cluster_centre3.d<<","<<cluster_centre3.theeta<<")"<<endl;
            
            dist1 = kmeans_dist(detected_lines[count], cluster_centre1);
            dist2 = kmeans_dist(detected_lines[count], cluster_centre2);
            dist3 = kmeans_dist(detected_lines[count], cluster_centre3);
            
            cout<<"dist1: "<<dist1<<endl;
            cout<<"dist2: "<<dist2<<endl;
            cout<<"dist3: "<<dist3<<endl;

            if(dist1<=dist2 && dist1<=dist3)
            {
                (lines_to_be_clustered[count]).cluster_no =  1;
                cout<<"Cluster no: 1"<<endl<<endl;
                temp_cluster_centre1.d = temp_cluster_centre1.d + ((lines_to_be_clustered[count]).line).d;
                temp_cluster_centre1.theeta = temp_cluster_centre1.theeta + ((lines_to_be_clustered[count]).line).theeta;
                no_cluster1++;              
            }

            else if(dist2<=dist1 && dist2<=dist3)
            {
                (lines_to_be_clustered[count]).cluster_no =  2;
                cout<<"Cluster no: 2"<<endl<<endl;
                temp_cluster_centre2.d = temp_cluster_centre2.d + ((lines_to_be_clustered[count]).line).d;
                temp_cluster_centre2.theeta = temp_cluster_centre2.theeta + lines_to_be_clustered[count].line.theeta;
                no_cluster2++;
            }

            else
            {
                (lines_to_be_clustered[count]).cluster_no = 3;
                cout<<"Cluster no: 3"<<endl<<endl;
                temp_cluster_centre3.d = temp_cluster_centre3.d + ((lines_to_be_clustered[count]).line).d;
                temp_cluster_centre3.theeta = temp_cluster_centre3.theeta + lines_to_be_clustered[count].line.theeta;
                no_cluster3++;
            }
            
        }

        //////////////////Upadating the cluster centres//////////////////
        cluster_centre1.d = ((double)temp_cluster_centre1.d)/no_cluster1;
        cluster_centre1.theeta = ((double)temp_cluster_centre1.theeta)/no_cluster1;
        cluster_centre2.d = (double)temp_cluster_centre2.d/no_cluster2;
        cluster_centre2.theeta = (double)temp_cluster_centre2.theeta/no_cluster2;
        cluster_centre3.d = (double)temp_cluster_centre3.d/no_cluster3;
        cluster_centre3.theeta = (double)temp_cluster_centre3.theeta/no_cluster3;

        cout<<"After "<<k_means_index<<"iterations, (cluster_centre1.d,cluster_centre1.theeta): ("<<cluster_centre1.d<<","<<cluster_centre1.theeta<<")"<<endl;
        cout<<"no_cluster1: "<<no_cluster1<<endl;
        
        cout<<"After "<<k_means_index<<"iterations, (cluster_centre2.d,cluster_centre2.theeta): ("<<cluster_centre2.d<<","<<cluster_centre2.theeta<<")"<<endl;
        cout<<"no_cluster2: "<<no_cluster2<<endl;

        cout<<"After "<<k_means_index<<"iterations, (cluster_centre3.d,cluster_centre3.theeta): ("<<cluster_centre3.d<<","<<cluster_centre3.theeta<<")"<<endl;
        cout<<"no_cluster3: "<<no_cluster3<<endl;
    }

    cout<<"FINALLY: cluster_centre1.d: "<<cluster_centre1.d<<"  "<<"cluster_centre1.theeta: "<<cluster_centre1.theeta<<endl; 
    cout<<"FINALLY: cluster_centre2.d: "<<cluster_centre2.d<<"  "<<"cluster_centre2.theeta: "<<cluster_centre2.theeta<<endl; 
    cout<<"FINALLY: cluster_centre3.d: "<<cluster_centre3.d<<"  "<<"cluster_centre3.theeta: "<<cluster_centre3.theeta<<endl; 

    cout<<endl<<endl<<"According to the input file, the 3 detected lines should be:"<<endl; 
    cout<<"x+y=10"<<endl;
    cout<<"x=8"<<endl;
    cout<<"x=y"<<endl<<endl;

    cout<<"From the noisy input from the file kd_data.txt, the 3 detected lines are: "<<endl;


    //3 if-else statements because assumed that there are only 3 lines in the image 
    if(sin(cluster_centre1.theeta)>=0)
        cout<<cos(cluster_centre1.theeta)<<"x"<<"+"<<sin(cluster_centre1.theeta)<<"y"<<"="<<cluster_centre1.d<<endl;
    else if(sin(cluster_centre1.theeta)<0)
        cout<<cos(cluster_centre1.theeta)<<"x"<<sin(cluster_centre1.theeta)<<"y"<<"="<<cluster_centre1.d<<endl;;
    
    if(sin(cluster_centre2.theeta)>=0)
        cout<<cos(cluster_centre2.theeta)<<"x"<<"+"<<sin(cluster_centre2.theeta)<<"y"<<"="<<cluster_centre2.d<<endl;
    else if(sin(cluster_centre2.theeta)<0)
        cout<<cos(cluster_centre2.theeta)<<"x"<<sin(cluster_centre2.theeta)<<"y"<<"="<<cluster_centre2.d<<endl;
    
    if(sin(cluster_centre3.theeta)>=0)
        cout<<cos(cluster_centre3.theeta)<<"x"<<"+"<<sin(cluster_centre3.theeta)<<"y"<<"="<<cluster_centre3.d<<endl;
    else if(sin(cluster_centre3.theeta)<0)
        cout<<cos(cluster_centre3.theeta)<<"x"<<sin(cluster_centre3.theeta)<<"y"<<"="<<cluster_centre3.d<<endl;

    return 0;
} 