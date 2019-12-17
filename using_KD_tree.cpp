/////////////kd_data.txt contains the x-coordinate followed by the y-coordinate of the points////////////////////
///////////CODE TO TAKE IN POINTS, USE KD_TREE DATA STRUCTURE, RANSAC ALGORITHM TO DETECT THE LINES//////////////
////////////////////Reading the text file, "kd_data.txt" for data points/////////////////////////////////////////
////////////////////kd_data.txt has 65 points lying roughly on the lines ( "x=8", "x+y=10", "x=y" )//////////////
#include<bits/stdc++.h> 
#include <iostream>
#include <fstream>//inlcude the fstream class
#include <math.h> 
#include <vector>
#include <stdlib.h>
#include <stdio.h>
using namespace std; 
  
const int k = 20; //for each point of the 2 randomly chosen points to define the line in RANSAC,
//look at the 20 nearest neighbours of the point and check whether they lie within a threshold distance
//of the formed line or not
  
struct Point 
{
    double x;
    double y;
};

// A structure to represent node of kd tree 
struct Node 
{ 
    Point point; // To store k dimensional point 
    Node *left, *right; 
}; 

Node* parent_pointer;//required in the delete function, because for example, to delete a leaf node
//only setting the leaf node to NULL wont be sufficient, we will have to set the pointer from the 
//parent to the node equal to NULL

double euc_dist(Point p1, Point p2)//Euclidean distance
{
    double EU_dist = sqrt( pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2) );
    return EU_dist;
}
  
// A method to create a node of K D tree 
struct Node* newNode(Point pt) 
{ 
    struct Node* temp = new Node; 
  
    temp->point.x = pt.x; 
    temp->point.y = pt.y;
    
    temp->left = temp->right = NULL; 
    return temp; 
} 
  
// This functino inserts a new node and returns root of modified tree 
// Depth- to decide whether we need to compare based on the x or the y values of the points
Node *insertRec(Node *root, Point pt, int depth) 
{ 
    if (root == NULL)
        return newNode(pt); 
    
    // Calculate current dimension (cd) of comparison 
    int cd = depth % 2; 
  
    // Compare the new point with root on current dimension 'cd' 
    // and decide whether to go on the left or right subtree 
    if(cd == 0)
    {
        if (pt.x < (root->point.x)) 
            root->left = insertRec(root->left, pt, depth + 1); 
        else
            root->right = insertRec(root->right, pt, depth + 1); 
    }
    
    else if(cd == 1)
    {
        if (pt.y < (root->point.y)) 
            root->left = insertRec(root->left, pt, depth + 1); 
        else
            root->right = insertRec(root->right, pt, depth + 1); 
    }

    return root; 
} 
  
// Function to insert a new point with given point in 
// KD Tree and return new root. It mainly uses above recursive 
// function "insertRec()" 
Node* insert(Node *root, Point point) 
{ 
    return insertRec(root, point, 0); 
} 
  
void printKD_tree(Node* root)
{
    if(root == NULL)
        return;

    Node* temp = root;
    printKD_tree(root->left);
    cout<<"("<<root->point.x<<","<<root->point.y<<")  ";
    printKD_tree(root->right);
}

//Function to return pointer to the node with the maximum value in a particular dimenstion, out of 3 input nodes
Node *maxNode(Node *n1, Node *n2, Node *n3, int d) 
{ 
    //cout<<"ENtered maxNode function"<<endl;
    Node *res = n1; 
    if(d==0)
    {
        if (n2 != NULL && n2->point.x > res->point.x) 
            res = n2; 
        if (n3 != NULL && n3->point.x > res->point.x) 
            res = n3;
    }

    if(d==1)
    {
        if (n2 != NULL && n2->point.y > res->point.y) 
            res = n2; 
        if (n3 != NULL && n3->point.y > res->point.y) 
            res = n3;
    }
 
    return res; 
}  

//Function to return pointer to the node with the mainimum value in a particular dimenstion, out of 3 input nodes
Node *minNode(Node *n1, Node *n2, Node *n3, int d) 
{ 
    //cout<<"Entered minNode function"<<endl;
    Node *res = n1; 
    if(d==0)
    {
        if (n2 != NULL && n2->point.x < res->point.x) 
            res = n2; 
        if (n3 != NULL && n3->point.x < res->point.x) 
            res = n3;
    }

    if(d==1)
    {
        if (n2 != NULL && n2->point.y < res->point.y) 
            res = n2; 
        if (n3 != NULL && n3->point.y < res->point.y) 
            res = n3;
    }
 
    return res; 
} 
  
// The parameter depth is used to determine current axis, we are on
Node *findMaxRec(Node* root, int d, unsigned depth) 
{ 
    // Base cases 
    if (root == NULL) 
        return NULL; 
    //cout<<"findMaxRec with root: ("<<root->point.x<<","<<root->point.y<<")"<<endl;  
    // Current dimension is computed using current depth and total 
    // dimensions (k) 
    unsigned cd = depth % 2; 
  
    // Compare point with root with respect to cd (Current dimension) 
    if(cd == d) 
    { 
        if (root->right == NULL) 
            return root; 
        return findMaxRec(root->right, d, depth+1); 
    } 
  
    // If current dimension is different then minimum can be anywhere 
    // in this subtree 
    return maxNode(root, findMaxRec(root->left, d, depth+1), findMaxRec(root->right, d, depth+1), d); 
} 
  
//Functino used finaMaxRec() function to return the maximum of d'th dimension 
Node *findMax(Node* root, int d) 
{ 
    // Pass current level or depth as 1 
    return findMaxRec(root, d, 0); 
} 


// Recursively finds minimum of d'th dimension in KD tree 
// The parameter depth is used to determine current axis we are on
// Function used finaMaxRec() function to return the maximum of d'th dimension 
Node *findMinRec(Node* root, int d, unsigned depth) 
{ 
    if (root == NULL) 
        return NULL;  
 
    unsigned cd = depth % 2; 
  
    // Compare point with root with respect to cd (Current dimension)
    //if the dimension is same then obviously the minimum will lie towards the left of the root 
    if(cd == d) 
    { 
        if (root->left == NULL) 
            return root; 
        return findMinRec(root->left, d, depth+1); 
    } 
  
    // If current dimension is different then minimum can be anywhere in this subtree
    return minNode(root, findMinRec(root->left, d, depth+1), findMinRec(root->right, d, depth+1), d); 
} 
  
// A wrapper over findMinRec(). Returns minimum of d'th dimension 
Node *findMin(Node* root, int d) 
{ 
    // Pass current level or depth as 1 
    return findMinRec(root, d, 1); 
} 
   

bool arePointsSame(Point point1, Point point2) 
{ 
    if (point1.x != point2.x  ||  point1.y != point2.y) 
        return false; 
  
    return true; 
} 
  
// Copies point p2 to p1 
void copyPoint(Point* p1, Point* p2) 
{ 
    (*p1).x = (*p2).x;
    (*p1).y = (*p2).y;  
} 

//function to search for the parent of the point in the KD_tree
Node* search_parentRec(Node* root, Point point, unsigned depth) 
{  
    if (root == NULL) 
        return NULL; 
    
    if(root->left==NULL && root->right==NULL)
        return root;

    if(root->left!=NULL && root->right!=NULL)
    {
        if( ( arePointsSame(root->left->point, point) ) || ( arePointsSame(root->right->point, point) ) )
            return root;
    }

    if(root->left==NULL && root->right!=NULL)
    {
        if( arePointsSame(root->right->point, point) )
            return root;
    }
    
    if(root->right==NULL && root->left!=NULL)
    {
        if( arePointsSame(root->left->point, point) )
            return root;
    }
    // Current dimension is computed using current depth and total 
    // dimensions (k) 
    unsigned cd = depth % 2; 

    if(cd==0)
    {
        if(point.x < root->point.x) 
            return search_parentRec(root->left, point, depth + 1);
        else
            return search_parentRec(root->right, point, depth + 1); 
    } 

    if(cd==1)
    {
        if(point.y < root->point.y) 
            return search_parentRec(root->left, point, depth + 1);
        else
            return search_parentRec(root->right, point, depth + 1); 
    }    
    
} 
  
// Searches a Point in the K D tree. It mainly uses searchRec() 
Node* search_parent(Node* root, Point point) 
{  
    Node* pp = search_parentRec(root, point, 0); 
    return pp;
} 


Node* searchRec(Node* root, Point point, unsigned depth) 
{ 
    // Base cases 
    if (root == NULL) 
        return NULL; 
    if (arePointsSame(root->point, point)) 
        return root; 
  
    // Current dimension is computed using current depth and total 
    // dimensions (k) 
    unsigned cd = depth % 2; 
    if(cd == 0)
    {
        if (point.x < root->point.x) 
            return searchRec(root->left, point, depth + 1); 
        else
            return searchRec(root->right, point, depth + 1);    
    }

    else if(cd == 1)
    {
        if (point.y < root->point.y) 
            return searchRec(root->left, point, depth + 1);
        else
            return searchRec(root->right, point, depth + 1);
    }
} 
  
// Searches a Point in the K D tree. It mainly uses searchRec() 
Node* search(Node* root, Point point) 
{ 
    // Pass current depth as 0 
    Node* ptr = searchRec(root, point, 0); 
    return ptr;
} 
  
// Function to delete a given point "Point point" from tree with root as 'root'. 
// This function Returns root of the modified tree. 
// The depth field is the depth of the root node 
Node *deleteNodeRec(Node* head_ref, Node *root, Point point, int depth) 
{ 
    //cout<<"Entered deleteNodeRec function with root: ("<<root->point.x<<","<<root->point.y<<")";
    //<<" and node to be deleted has the point: ("<<point.x<<","<<point.y<<")"<<endl;
    // Given point is not present 
    if (root == NULL) 
        return NULL; 

    // Find dimension of current node 
    int cd = depth % 2; 
  
    // If the point to be deleted is present at root 
    if (arePointsSame(root->point, point)) 
    { 
        // right subtree exists
        if (root->right != NULL) 
        { 
            Node *min = findMin(root->right, cd); 
            
            copyPoint(&(root->point), &(min->point)); 
            
            //keep updating the parent_pointer, which will be needed when we encounter a leaf node,
            //that is when we encounter the base case of the delete function
            parent_pointer = root;
            // Recursively delete the minimum of the right subtree 
            root->right = deleteNodeRec(head_ref, root->right, min->point, depth+1); 
        } 

        else if (root->left != NULL) // if right child is null, but left subtree exists...same as above 
        { 
            //Left subtree exists
            //find the node with the maximum value of the current dimension in the left subtree  
            Node *max = findMax(root->left, cd); 
            
            copyPoint(&(root->point), &(max->point)); 
            parent_pointer = root;//update the parent pointer
            // Recursively delete the maximum of the left subtree
            root->left = deleteNodeRec(head_ref, root->left, max->point, depth+1); 
        }

        else // If node to be deleted is leaf node 
        { 
            if(parent_pointer->left!=NULL)
            {
                if(arePointsSame(parent_pointer->left->point,point))
                    parent_pointer->left = NULL;
                else if(arePointsSame(parent_pointer->right->point,point))
                    parent_pointer->right = NULL;
            }

            else if(parent_pointer->right!=NULL)
            {
                if(arePointsSame(parent_pointer->right->point,point))
                    parent_pointer->right = NULL;
                
                else if(arePointsSame(parent_pointer->left->point,point))
                    parent_pointer->left = NULL; 
            }

            parent_pointer = root;
            delete root;
            return NULL; 
        } 
        
        return root; 
    } 
  
    //update the parent_pointer
    parent_pointer = root;

    if(cd==0)
    {
        if (point.x < root->point.x) 
        {
            //cout<<"root->right = deleteNodeRec()"<<endl;
            root->left = deleteNodeRec(head_ref, root->left, point, depth+1); 
        }
       
        else
        {
            //cout<<"root->right = deleteNodeRec()"<<endl;
            root->right = deleteNodeRec(head_ref, root->right, point, depth+1);
        }  
    }

    else if(cd==1)
    {
        if (point.y < root->point.y)
            root->left = deleteNodeRec(head_ref, root->left, point, depth+1); 
        
        else
        {
            //cout<<"root->right = deleteNodeRec()"<<endl;
            root->right = deleteNodeRec(head_ref, root->right, point, depth+1);
        }  
    }
    
    return root; 
} 
  
// Function to delete a given point from K D Tree with 'root' 
Node* deleteNode(Node *root, Point point) 
{ 
   // Pass depth as 0 
   return deleteNodeRec(root, root, point, 0); 
} 


vector<Point> points;
vector<Point> KNN_list;//this is the list of the KNN points of the 2 query points which will be selected 
//randomly in RANSAC

double w = 10000;
Point* p;
Node* pointer_to_nearest_neighbor_node;

//initial call NNS(q,root,p,infintiy,0)
void NNS(Node* header_root, Point q, Node* n, int depth)
{
    if(n == NULL)
        return;
    
    double w_temp;
    int cd = depth%2; //cd: current dimension
    int search_first;//0 if left, 1 if right has to be searched first
    
    if(n->left == NULL && n->right == NULL)//leaf node encountered
    {
        w_temp = euc_dist(q, n->point);
       
        if(w_temp < w)
        {
            w = w_temp;
            p = &(n->point);
            pointer_to_nearest_neighbor_node = n;
            //cout<<"w updated to: "<<w<<endl;
            //cout<<"p updated to: ("<<(*p).x<<","<<(*p).y<<")"<<endl;
        }
    }
 
    else//Not a leaf node
    {
        w_temp = euc_dist(q,n->point);

        if(w_temp < w)
        {
            w = w_temp;
            p = &(n->point);
            //cout<<"w updated to: "<<w<<endl;
            //cout<<"p updated to: ("<<(*p).x<<","<<(*p).y<<")"<<endl;
        }

        if(cd == 0)
        {
            if(q.x < (n->point).x)
                search_first = 0;
            else
                search_first = 1;//for same values we move to the right node    

            if(search_first == 0)
            {
                //cout<<"Searching in the left side of ("<<n->point.x<<","<<n->point.y<<") with root: ("<<n->point.x<<","<<n->point.y<<")"<<endl;
                NNS(header_root, q, n->left,  depth+1);

                if((n->point).x - q.x < w)//it is worthwhile to check the bad side(right side) of n also in this case
                {
                    //cout<<"Worth Checking the right side of ("<<n->point.x<<","<<n->point.y<<") with root: ("<<n->point.x<<","<<n->point.y<<")"<<endl;
                    NNS(header_root, q, n->right,  depth+1);
                }

                //else
                    //cout<<"Not worth checking the right side of ("<<n->point.x<<","<<n->point.y<<") with root: ("<<n->point.x<<","<<n->point.y<<")"<<endl;
            }
           
            else
            {
                //cout<<"Searching in the right side of ("<<n->point.x<<","<<n->point.y<<") with root: ("<<n->point.x<<","<<n->point.y<<")"<<endl;
                NNS(header_root, q,n->right, depth+1);

                if(q.x - (n->point).x < w)//it is worthwhile to check the bad side(left side) of n also in this case
                {
                    //cout<<"Worth checking the left side of ("<<n->point.x<<","<<n->point.y<<") with root: ("<<n->point.x<<","<<n->point.y<<")"<<endl;
                    NNS(header_root, q,n->left, depth+1);
                }
                
                //else 
                    //cout<<"Not worth checking the left side of ("<<n->point.x<<","<<n->point.y<<") with root: ("<<n->point.x<<","<<n->point.y<<")"<<endl;
            }
        }

        else if(cd == 1)
        {
            if(q.y < (n->point).y)
                search_first = 0;
            else
                search_first = 1;//for same values we move to the right node    

            if(search_first == 0)
            {
                //cout<<"Searching in the left side of ("<<n->point.x<<","<<n->point.y<<") with root: ("<<n->point.x<<","<<n->point.y<<")"<<endl;
                NNS(header_root, q, n->left, depth+1);

                if((n->point).y - q.y < w)//it is worthwhile to check the bad side of n also in this case
                {
                    //cout<<"Worth checking the right side of ("<<n->point.x<<","<<n->point.y<<") with root: ("<<n->point.x<<","<<n->point.y<<")"<<endl;
                    NNS(header_root, q,n->right, depth+1);
                }    
                
               //else
                    //cout<<"Not worth checking the right side of ("<<n->point.x<<","<<n->point.y<<") with root: ("<<n->point.x<<","<<n->point.y<<")"<<endl;
            }
           
            else
            {
                //cout<<"Searching in the right side of ("<<n->point.x<<","<<n->point.y<<") with root: ("<<n->point.x<<","<<n->point.y<<")"<<endl;
                NNS(header_root, q,n->right,depth+1);

                if(q.y - (n->point).y < w)//it is worthwhile to check the bad side of n also in this case
                {
                    //cout<<"Worth checking the left side of ("<<n->point.x<<","<<n->point.y<<") with root: ("<<n->point.x<<","<<n->point.y<<")"<<endl;
                    NNS(header_root, q,n->left, depth+1);
                }
                
                //else 
                    //cout<<"Not worth checking the left side of ("<<n->point.x<<","<<n->point.y<<") with root: ("<<n->point.x<<","<<n->point.y<<")"<<endl;
            }
        }

    }

    return;
}


void KNNS(Node* header_root, Point q, Node* n, int depth)//K is the K in K nearest neighbours
{
    int i;
    int curr_depth;//if this is zero for the p obtained, it means that compare by x values, otherwise compare by y value
    Point temp_p;
    parent_pointer = header_root;
    
    for(i=0;i<k;i++)
    {
        w = 10000;
        cout<<"KNN iteration number: "<<i<<endl;
        NNS(header_root, q, n, depth);
        //"pointer_to_nearest_neighbor_node"- delete the node to which this pointer is pointing to
       
        temp_p = *p;
        cout<<"With query point: ("<<q.x<<","<<q.y<<"), ";
        cout<<"the nearest point: ("<<temp_p.x<<","<<temp_p.y<<")"<<endl;
        KNN_list.push_back(temp_p);
        cout<<"The size of KNN_list is: "<<KNN_list.size()<<endl;
        
        deleteNode(header_root, temp_p);//delete the next nearest neighbour of the query point
        //so that in the next iteration of KNNS, the same node is not detected as the nearest neighbour
        cout<<"The KD tree after "<<i<<" iterations: "<<endl;
        printKD_tree(n);
        cout<<endl<<endl;
    }
}


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


int main()
{
    vector<Point> Pts;//this is the list of points from the input text file

    Point temp_point;
    bool flag = 0;

    vector<line_datatype> detected_lines;//list of all the detected lines from RANSAC and using KD_tree
    vector<dataset> lines_to_be_clustered;
    dataset data;

    int index, counter, count, RANSAC_iterations, threshold_for_line, k_means_iteration, no_cluster1, no_cluster2, no_cluster3, k_means_index, no_detected_lines;
    double theeta, d, dist_threshold, dist1, dist2, dist3;//Point normal form of a line: "d = x*cos(theeta) + y*sin(theeta)" has been used
    Point pt1, pt2;//we will randomly select 2 points for RANSAC algo 'iterations' number of times
    line_datatype line;

    srand(time(0));
    //no_points = Pts.size();
    k_means_iteration = 1000;//we use 100 iterations of the K-means algo
    RANSAC_iterations = 1000;//the number of times we want to randomly test for a line...ideally we want a big number like 100 or above, but becoming computationally expensive 
    threshold_for_line = 20;//minimum number of points to lie on a line, for the line to be classified as a valid line
    //this parameter is dependent of the number of points we have in the input
    dist_threshold = 2;//maximum distance upto which a point may lie from a line for that point to be considered a part of the line
    
    no_cluster1 = no_cluster2 = no_cluster3 = 0;

    int rand1, rand2, rand3;
    int no_points;
    int no_points_in_KNN_list;
    int i, j;

    struct Node *root;

    ifstream inFile;//inFile is an object of the ifstream class 
    inFile.open("kd_data.txt");

    //check for Error
    if(inFile.fail())
    {
        cerr<<"Error opening file"<<endl;
        exit(1);
    }


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

    no_points = Pts.size();
    cout<<no_points<<"points detected from the text file"<<endl;
      

//Since we are tampering with the KD_tree, 
//Hence, before each iteration of this loop, we will have to build a fresh KD_tree
//Hence, in each iteration of the RANSAC loop, create a KD_tree with a fresh root, taking inputs from the pts vector
    for(count=0;count<RANSAC_iterations;count++)
    {
        root = NULL; 
        cout<<endl<<endl<<endl<<count<<" iteration of RANSAC"<<endl<<endl<<endl<<endl;

        for (int i=0; i<no_points; i++) 
           root = insert(root, Pts[i]);

        cout<<"The KD tree looks as follows"<<endl;
        printKD_tree(root);
        cout<<endl;

        KNN_list.clear();
        counter = 0;

        //cout<<"Iteration: "<<count<<endl;
        rand1 = rand()%no_points;
        rand2 = rand()%no_points;

        if(rand1 == rand2)
            rand2 = rand1+1;

        pt1 = Pts[rand1];
        pt2 = Pts[rand2];

        cout<<"pt1: ("<<pt1.x<<","<<pt1.y<<")"<<endl;
        cout<<"pt2: ("<<pt2.x<<","<<pt2.y<<")"<<endl;


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

        

        KNNS(root, pt1, root, 0);
        no_points_in_KNN_list = KNN_list.size();
        //now KNN_list has all the points which are the K-neighbours of pt1
        //hence, now check whether these points are close enough to the line or not

        for(index=0;index<no_points_in_KNN_list;index++)
        {
            cout<<"KNN_list["<<index<<"]: ("<<KNN_list[index].x<<","<<KNN_list[index].y<<")"<<endl;
            if(dist(KNN_list[index],line) < dist_threshold)
                counter++;
        }

        KNN_list.clear();

        cout<<endl<<endl<<"Moving on to KNNs of pt2: ("<<pt2.x<<","<<pt2.y<<")"<<endl;
        root = NULL;
        
        //make a fresh KD_tree for pt2
        for (int i=0; i<no_points; i++) 
           root = insert(root, Pts[i]);

        cout<<"The KD tree looks as follows"<<endl;
        printKD_tree(root);
        cout<<endl;

        KNNS(root, pt2, root, 0);
        no_points_in_KNN_list = KNN_list.size();

        for(index=0;index<no_points_in_KNN_list;index++)
        {
            cout<<"KNN_list["<<index<<"]: ("<<KNN_list[index].x<<","<<KNN_list[index].y<<")"<<endl;
            if(dist(KNN_list[index],line) < dist_threshold)
                counter++;
        }

        //the counter value now gives us the number of points within the k-neighbourhood of both the points and lying close enough to the line formed by the 2 points

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

//The lines have been detected perfectly......
//Now we will cluster the lines into groups of 3(Assumed we know the number of lines beforehand)
    cout<<"The detected_lines are: "<<endl<<endl;
    
    ////////////Now comes the code for clustering of the lines//////////
    ////Assumption: There are 3 lines being formed from the data set////
    
    rand1 = rand()%(no_detected_lines-1);
    rand2 = rand()%(no_detected_lines-1);
    rand3 = rand()%(no_detected_lines-1);

    //So that rand1,2,3 are all different so that 2 selected cluster centres are not the same
    //Load: Even though rand1, rand2, rand3 are different, the lines obtained may be the same....
    //Whereas we want the 3 cluster centres selected initially randomly, to all be in the 3 different clusters
    while(rand1 == rand2 || rand1 == rand3 || rand2 == rand3)
    {
        rand1 = rand()%(no_detected_lines-1);
        rand2 = rand()%(no_detected_lines-1);
        rand3 = rand()%(no_detected_lines-1);
    }

    cout<<"rand1: "<<rand1<<endl;
    cout<<"rand2: "<<rand2<<endl;
    cout<<"rand3: "<<rand3<<endl;

    //randomly initialising the 3 cluster centres
    line_datatype cluster_centre1 = detected_lines[rand1];
    line_datatype cluster_centre2 = detected_lines[rand2];
    line_datatype cluster_centre3 = detected_lines[rand3];

    line_datatype temp_cluster_centre1;
    line_datatype temp_cluster_centre2;
    line_datatype temp_cluster_centre3;
    //double c1d, c1theeta, c2d, c2theeta, c3d, c3theeta; 
    
    //this for loop is for initialising "data" which is an object of the class dataset
    for(count=0;count<no_detected_lines;count++)
    {
        cout<<"count: "<<count<<endl;
        data.line = detected_lines[count];
        //data is an object of the data type "data_type", which contains both the line details and the cluster number details

        cout<<"detected_lines["<<count<<"]"<<"("<<detected_lines[count].d<<","<<detected_lines[count].theeta<<")"<<endl;
        cout<<"cluster_centre1: ("<<cluster_centre1.d<<","<<cluster_centre1.theeta<<")"<<endl;
        cout<<"cluster_centre2: ("<<cluster_centre2.d<<","<<cluster_centre2.theeta<<")"<<endl;
        cout<<"cluster_centre3: ("<<cluster_centre3.d<<","<<cluster_centre3.theeta<<")"<<endl;

        //assigning cluster numbers to all the points
        dist1 = kmeans_dist(detected_lines[count], cluster_centre1);
        dist2 = kmeans_dist(detected_lines[count], cluster_centre2);
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

        //assigning cluster numbers to all the detected lines and counting the total number of points belonging to each cluster
        for(count=0;count<no_detected_lines;count++)
        {
            cout<<"count: "<<count<<endl;

            cout<<"detected_lines["<<count<<"]"<<"("<<detected_lines[count].d<<","<<detected_lines[count].theeta<<")"<<endl;
            cout<<"cluster_centre1: ("<<cluster_centre1.d<<","<<cluster_centre1.theeta<<")"<<endl;
            cout<<"cluster_centre2: ("<<cluster_centre2.d<<","<<cluster_centre2.theeta<<")"<<endl;
            cout<<"cluster_centre3: ("<<cluster_centre3.d<<","<<cluster_centre3.theeta<<")"<<endl;

            //assigning cluster numbers to all the lines detected
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

        cluster_centre1.d = ((double)temp_cluster_centre1.d)/no_cluster1;
        cluster_centre1.theeta = ((double)temp_cluster_centre1.theeta)/no_cluster1;
        //cout<<"("<<c1d<<","<<c1theeta<<")"<<endl;

        cluster_centre2.d = (double)temp_cluster_centre2.d/no_cluster2;
        cluster_centre2.theeta = (double)temp_cluster_centre2.theeta/no_cluster2;
        //cout<<"("<<c2d<<","<<c2theeta<<")"<<endl;

        cluster_centre3.d = (double)temp_cluster_centre3.d/no_cluster3;
        cluster_centre3.theeta = (double)temp_cluster_centre3.theeta/no_cluster3;
        //cout<<"("<<c3d<<","<<c3theeta<<")"<<endl;

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