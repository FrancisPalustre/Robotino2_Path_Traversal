#include <iostream>
#include <vector>

//Time Libraries
#include <sys/time.h>

//Random number generation 
#include <cstdlib> 
#include <unistd.h>
#include <sys/time.h>

//Dijkstra Libraries
#include <queue>
#include <algorithm>
#include <map>
#include <limits>

//Robotino Libraries
#include "rec/robotino/api2/all.h" 

using namespace rec::robotino::api2;
using namespace std;

class MyCom : public Com {
public:
	MyCom():Com( "Dijkstra" ){} //naming com after our program

	void errorEvent( const char* errorString ) {
		cerr << "Error: " << errorString << endl;
	}

	void connectedEvent() { //indicating that we are connected w/ Robotino
		cout << "Connected." << endl;
	}

	void connectionClosedEvent() { //terminated our connection w/ Robotino
		cout << "Connection closed." << endl;
	}
};

OmniDrive omni; //global so we can access omni.velocity throughout program

//Get current time in seconds
double getCurrentTime() {
    struct timeval tv; //timeval struct to store time values
    gettimeofday(&tv, NULL); //get current time and store in timeval

    //convert time to seconds and add microseconds for precision
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

//Robotino movement in X-direction
void moveX(int curr_X, int desired_X) {
    int checker = desired_X - curr_X; //determines if movement is positive or negative
    int distance = abs(checker); //how much travel is needed

    double start_time = getCurrentTime();
    double end_time = (distance * 1.635)+ start_time; //offset of 1.772 is used (can change to whatever is yours)
    double elapsed_time = 0;

    for (int i =0; i < distance;i++) {
        if (checker > 0) { //if positive...
            elapsed_time = 0; //resetting time to avoid continous movement
            do {
                elapsed_time = getCurrentTime(); //updating elasped time
                omni.setVelocity(0.0033,-0.2,-0.007); //velocity of Robotino (adjust accordingly)
                usleep(100000); //adds delay to avoid processing timing problems
            } while (elapsed_time < end_time);
        } else { //if negative..
            elapsed_time = 0;
            do {
                elapsed_time = getCurrentTime();
                omni.setVelocity(-0.00151,0.2,0);
                usleep(100000);
            } while (elapsed_time < end_time);
        }
    }
    omni.setVelocity(0,0,0); //setting velocity to 0 for precautionary sake
}

//Robotino movement in Y-direction
void moveY(int curr_Y, int desired_Y) { //same comments applied to this function
    int checker = desired_Y - curr_Y;
    int distance = abs(checker);

    double start_time = getCurrentTime();
    double end_time = (distance * 1.25)+ start_time;
    double elapsed_time = 0;

    for (int i =0; i < distance;i++) {
        if (checker > 0) {
            elapsed_time = 0;
            do {
                elapsed_time = getCurrentTime();
                omni.setVelocity(0.25,-0.00135,0);
                usleep(100000);
            } while (elapsed_time < end_time);
        } else {
            elapsed_time = 0;
            do {
                elapsed_time = getCurrentTime();
                omni.setVelocity(-0.2,0.01,0);
               usleep(100000);
            } while (elapsed_time < end_time);
        }
    }
    omni.setVelocity(0,0,0);
}

//Only purpose is to be able to collapse graph due to having 36 nodes
map<string, map<string, int> > graphFunction() {
	//AL of current 8x8 Graph
    map<string, map<string, int> > graph;

    graph["1"]["2"] = 1; graph["1"]["7"] = 1;
    graph["2"]["1"] = 1; graph["2"]["8"] = 1; graph["2"]["3"] = 1;
    graph["3"]["2"] = 1; graph["3"]["9"] = 1; graph["3"]["4"] = 1;
    graph["4"]["3"] = 1; graph["4"]["10"] = 1; graph["4"]["5"] = 1;
    graph["5"]["4"] = 1; graph["5"]["11"] = 1; graph["5"]["6"] = 1;
    graph["6"]["5"] = 1; graph["6"]["12"] = 1;

    graph["7"]["13"] = 1; graph["7"]["8"] = 1; graph["7"]["1"] = 1;
    graph["8"]["14"] = 1; graph["8"]["9"] = 1; graph["8"]["2"] = 1; graph["8"]["7"] = 1;
    graph["9"]["15"] = 1; graph["9"]["10"] = 1; graph["9"]["3"] = 1; graph["9"]["8"] = 1;
    graph["10"]["16"] = 1; graph["10"]["11"] = 1; graph["10"]["4"] = 1; graph["10"]["9"] = 1;
    graph["11"]["17"] = 1; graph["11"]["12"] = 1; graph["11"]["5"] = 1; graph["11"]["10"] = 1;
    graph["12"]["18"] = 1; graph["12"]["11"] = 1; graph["12"]["6"] = 1;

    graph["13"]["19"] = 1; graph["13"]["14"] = 1; graph["13"]["7"] = 1;
    graph["14"]["20"] = 1; graph["14"]["15"] = 1; graph["14"]["8"] = 1; graph["14"]["13"] = 1;
    graph["15"]["21"] = 1; graph["15"]["16"] = 1; graph["15"]["9"] = 1; graph["15"]["14"] = 1;
    graph["16"]["22"] = 1; graph["16"]["17"] = 1; graph["16"]["10"] = 1; graph["16"]["15"] = 1;
    graph["17"]["23"] = 1; graph["17"]["18"] = 1; graph["17"]["11"] = 1; graph["17"]["16"] = 1;
    graph["18"]["24"] = 1; graph["18"]["17"] = 1; graph["18"]["12"] = 1;

    graph["19"]["25"] = 1; graph["19"]["20"] = 1; graph["19"]["13"] = 1;
    graph["20"]["26"] = 1; graph["20"]["21"] = 1; graph["20"]["14"] = 1; graph["20"]["19"] = 1;
    graph["21"]["27"] = 1; graph["21"]["22"] = 1; graph["21"]["15"] = 1; graph["21"]["20"] = 1;
    graph["22"]["28"] = 1; graph["22"]["23"] = 1; graph["22"]["16"] = 1; graph["22"]["21"] = 1;
    graph["23"]["29"] = 1; graph["23"]["24"] = 1; graph["23"]["17"] = 1; graph["23"]["22"] = 1;
    graph["24"]["30"] = 1; graph["24"]["23"] = 1; graph["24"]["18"] = 1;

    graph["25"]["31"] = 1; graph["25"]["26"] = 1; graph["25"]["19"] = 1;
    graph["26"]["32"] = 1; graph["26"]["27"] = 1; graph["26"]["20"] = 1; graph["26"]["25"] = 1;
    graph["27"]["33"] = 1; graph["27"]["28"] = 1; graph["27"]["21"] = 1; graph["27"]["26"] = 1;
    graph["28"]["34"] = 1; graph["28"]["29"] = 1; graph["28"]["22"] = 1; graph["28"]["27"] = 1;
    graph["29"]["34"] = 1; graph["29"]["30"] = 1; graph["29"]["23"] = 1; graph["29"]["28"] = 1;
    graph["30"]["36"] = 1; graph["30"]["29"] = 1; graph["30"]["24"] = 1;

    graph["31"]["32"] = 1; graph["31"]["23"] = 1; 
    graph["32"]["31"] = 1; graph["32"]["26"] = 1; graph["32"]["33"] = 1; 
    graph["33"]["32"] = 1; graph["33"]["27"] = 1; graph["33"]["34"] = 1; 
    graph["34"]["33"] = 1; graph["34"]["28"] = 1; graph["34"]["35"] = 1; 
    graph["35"]["34"] = 1; graph["35"]["29"] = 1; graph["35"]["36"] = 1; 
    graph["36"]["35"] = 1; graph["36"]["30"] = 1;

    return graph;
}

//Generate random weights to nodes for Dijkstra
map<string, map<string, int> > randWeights (map<string, map<string, int> > graph) {
    srand(time(0)); //generates new numbers for every execution based on node
    map<string, map<string, int> >::iterator node;

    for (node = graph.begin(); node != graph.end(); node++) { //for every node in the graph...
        map<string, int>::iterator adjNodes;
        for (adjNodes = node->second.begin(); adjNodes != node->second.end(); adjNodes++) { //for every neighbor node, referencing the value of the pair 
            adjNodes->second = rand() % 100; // generates a random number between 1-99
        }
    }
    return graph;
}

//Dijkstra Algorithm
vector<string> Dijkstra(map<string, map<string, int> > graph, string startNode, string desiredNode) {
    priority_queue<pair<int, string>, vector<pair<int, string> >, greater<pair<int, string> > > pq; //stores smallest distance
    
    //store the shortest distance found
    map<string, int> distances;
    
    //store the predecessor of shortest path
    map<string, string> predecessors;
    
    vector<string> visited; //store visited
    
    //initialize distances to infinity and set distance of start node to 0
    map<string, map<string, int> >::const_iterator node = graph.begin();
    
    for (node; node != graph.end(); node++) {
        distances[node->first] = numeric_limits<int>::max();
    } distances[startNode] = 0;
    
    pq.push(make_pair(0, startNode)); //add start node to pq w/ initial value of 0
    
    while (!pq.empty()) {
        //get the node's value with the smallest distance from the priority queue
        string currNode = pq.top().second;
        pq.pop();
        
        //skips if node already has been visited
        vector<string>::iterator visitedNode = find(visited.begin(), visited.end(), currNode);
        if (visitedNode != visited.end()) {
            continue;
        }
        
        //mark this node as visited
        visited.push_back(currNode);
        map<string, int>::const_iterator neighbor;
        for (neighbor = graph[currNode].begin(); neighbor != graph[currNode].end(); neighbor++) { //for every neighbor of currNode..
            
            string neighborNode = neighbor->first; //grabbing neighboring node of pair
            int edgeWeight = neighbor->second; //grabbing value of neighbor from pair
            int newDistance = distances[currNode] + edgeWeight; //calculates that changed distance
            
            //if newDistance is less than current distance of neighbor
            if (newDistance < distances[neighborNode]) {
                
                //updating values w/ new set
                distances[neighborNode] = newDistance;
                predecessors[neighborNode] = currNode;
                
                pq.push(make_pair(newDistance, neighborNode)); //storing new nodes into queue
            }
        }
    }
    
    //reorganizes path since current is last to first, converting to first to last
    vector<string> path;
    for (string curr = desiredNode; curr != ""; curr = predecessors[curr]) {
        path.push_back(curr);
    }
    reverse(path.begin(), path.end());
    return path;
}

//Print path and movements for a robot to move from start to desired node
void roboMove(map<string, map<string, int> > graph, string startNode, string desiredNode) {
	vector<string> carPath =  Dijkstra(graph, startNode, desiredNode);

	//couts path
    for (vector<string>::iterator it = carPath.begin(); it != carPath.end(); it++) {
        cout << *it << " ";
    } cout << endl;

    int currPos =0;
    int nextPos =0;

    //Movement of Robotino
    for (int i = 0; i < int(carPath.size()) - 1; i++) {
        //Position is based on current and next value in path
        currPos = atoi(carPath.at(i).c_str());
        nextPos = atoi(carPath.at(i + 1).c_str());

        if ((currPos + 6) == nextPos) {
            moveY(0,1);
        }

        if ((currPos - 6) == nextPos) {
            moveY(1,0);
        }

        if ((currPos - 1) == nextPos) {
            moveX(1,0);
        }

        if ((currPos + 1) == nextPos) {
            moveX(0,1);
        }
    }
    cout <<endl;
    cout << carPath.back();

    omni.setVelocity(0,0,0);
}

//Remove edges of wanted nodes
map<string, map<string, int> > removeEdges(map<string, map<string, int> > graph ,vector<string> removeVec) {
    vector<string>::const_iterator currNode;
	for (currNode = removeVec.begin(); currNode != removeVec.end(); currNode++) {  //for every node in the removeVec...
        const string& node = *currNode; //node to remove edges from

        map<string, int>::const_iterator edge;
        for (edge = graph[node].begin(); edge != graph[node].end(); edge++) {  //for every edge of the current node... (auto is used so I dont have to type out iterator)
            const string& adjNode = edge->first;  //get the adjacent node's key
            graph[adjNode].erase(node);  //remove the edge from the adjacent node to the current node
        }
        graph[node].clear();  //remove all edges from the current node
    }
	return graph;
}

int main(int argc, char **argv) {
    MyCom com;

    if(argc > 1){
        com.setAddress(argv[1]);
        cout << "Trying to connect to " << argv[1] << "..." << endl;
    }else {
        com.setAddress("172.26.201.2");
        //com.setAddress("10.69.96.220");
        cout << "Trying to connect..." << endl;
    }
    try{
        com.connectToServer(true);
        cout << "Successfully connected!" << endl;
    } catch(...) {
        cout << "Unable to connect! Program stopped." << endl;
        return 0;
    }
    //sleep(5);
	//setting up graph
    map<string, map<string, int> > graph = graphFunction();

	//input variables
    string startingNode = "1";
    string desiredNode = "36";
    int nodeRemoval =0;
    sleep(7);
        
    // cout << "Starting Node: ";
    // cin >> startingNode;
        
    // cout << "Desired Node: ";
    // cin >> desiredNode;

    // cout << "\nHow many nodes to remove: ";
    // cin >> nodeRemoval;
    
	// //vector that houses the nodes/edges you want to remove from graph
    // vector <string> removeVec;
    // string input = "";
    
    // if (nodeRemoval != 0) {
    //     cout << "\nList:" <<endl;
    // }
    
    // for (int i =0; i < nodeRemoval; i++) {
    //     cout << "Node " << i+1 << ": ";
    //     cin >> input;
    //     removeVec.push_back(input);    
    // }

	//resultant graph after removing edges
	//graph = removeEdges(graph, removeVec);

	//resultant graph with the new weighted values
	graph = randWeights(graph);

	roboMove(graph, startingNode, desiredNode);

    return 0;
}