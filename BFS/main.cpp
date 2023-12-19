#include <iostream>
#include <vector>

//Time Libraries
#include <cstdlib>
#include <unistd.h>
#include <sys/time.h>

//BFS Libraries
#include <map>
#include <queue>
#include <algorithm> 

//Robotino Libraries
#include "rec/robotino/api2/all.h" 

using namespace rec::robotino::api2;
using namespace std;

class MyCom : public Com {
public:
	MyCom():Com( "BFS" ){} //naming com after our program

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
    double end_time = (distance * 1.772)+ start_time; //offset of 1.772 is used (can change to whatever is yours)
    double elapsed_time = 0;
 
    for (int i =0; i < distance;i++) {
        if (checker > 0) { //if positive...
            elapsed_time = 0; //resetting time to avoid continuous movement
            do {
                elapsed_time = getCurrentTime(); //updating elasped time
                omni.setVelocity(0.0015,-0.2,0); //velocity of Robotino (adjust accordingly)
               usleep(10000); //slight delay needed to avoid missing data
            } while (elapsed_time < end_time);
        } else { //if negative..
            elapsed_time = 0;
            do {
                elapsed_time = getCurrentTime();
                omni.setVelocity(-0.00151,0.2,0);
                usleep(10000);
             } while (elapsed_time < end_time);
     }
     omni.setVelocity(0,0,0); //setting velocity to 0 for precautionary sake
}

//Robotino movement in Y-direction
void moveY(int curr_Y, int desired_Y) { //same comments applied to this function
    int checker = desired_Y - curr_Y;
    int distance = abs(checker);
 
    double start_time = getCurrentTime();
    double end_time = (distance * 1.772)+ start_time;
    double elapsed_time = 0;
 
    for (int i =0; i < distance;i++) {
        if (checker > 0) {
            elapsed_time = 0;
            do {
                elapsed_time = getCurrentTime();
                omni.setVelocity(0.2,-0.00131,0);
                usleep(10000);
            } while (elapsed_time < end_time);
        } else {
            elapsed_time = 0;
            do {
                elapsed_time = getCurrentTime();
                omni.setVelocity(-0.2,0.01,0);
                usleep(10000);
            } while (elapsed_time < end_time);
        }
    }
    omni.setVelocity(0,0,0);
}

//Only purpose is to be able to collapse graph due to having 36 nodes
map<string, vector<string> > graphFunction() {
    map<string, vector<string> > graph;

    graph.insert(make_pair("1", vector<string>()));
    graph["1"].push_back("2"); graph["1"].push_back("7");

    graph.insert(make_pair("2", vector<string>()));
    graph["2"].push_back("1"); graph["2"].push_back("8"); graph["2"].push_back("3");

    graph.insert(make_pair("3", vector<string>()));
    graph["3"].push_back("2"); graph["3"].push_back("9"); graph["3"].push_back("4");

    graph.insert(make_pair("4", vector<string>()));
    graph["4"].push_back("3"); graph["4"].push_back("10"); graph["4"].push_back("5");

    graph.insert(make_pair("5", vector<string>()));
    graph["5"].push_back("4"); graph["5"].push_back("11"); graph["5"].push_back("6");

    graph.insert(make_pair("6", vector<string>()));
    graph["6"].push_back("5"); graph["6"].push_back("12");

    graph.insert(make_pair("7", vector<string>()));
    graph["7"].push_back("1"); graph["7"].push_back("13"); graph["7"].push_back("8");

    graph.insert(make_pair("8", vector<string>()));
    graph["8"].push_back("2"); graph["8"].push_back("7"); graph["8"].push_back("14"); graph["8"].push_back("9");

    graph.insert(make_pair("9", vector<string>()));
    graph["9"].push_back("3"); graph["9"].push_back("8"); graph["9"].push_back("15"); graph["9"].push_back("10");
    
    graph.insert(make_pair("10", vector<string>()));
    graph["10"].push_back("4"); graph["10"].push_back("9"); graph["10"].push_back("16"); graph["10"].push_back("11");

    graph.insert(make_pair("11", vector<string>()));
    graph["11"].push_back("5"); graph["11"].push_back("10"); graph["11"].push_back("17"); graph["11"].push_back("12");

    graph.insert(make_pair("12", vector<string>()));
    graph["12"].push_back("6"); graph["12"].push_back("11"); graph["12"].push_back("18");

    graph.insert(make_pair("13", vector<string>()));
    graph["13"].push_back("7"); graph["13"].push_back("14");graph["13"].push_back("19");

    graph.insert(make_pair("14", vector<string>()));
    graph["14"].push_back("8"); graph["14"].push_back("13"); graph["14"].push_back("15"); graph["14"].push_back("20");

    graph.insert(make_pair("15", vector<string>()));
    graph["15"].push_back("9"); graph["15"].push_back("14"); graph["15"].push_back("16"); graph["15"].push_back("21");

    graph.insert(make_pair("16", vector<string>()));
    graph["16"].push_back("10"); graph["16"].push_back("15"); graph["16"].push_back("17"); graph["16"].push_back("22");

    graph.insert(make_pair("17", vector<string>()));
    graph["17"].push_back("11"); graph["17"].push_back("16"); graph["17"].push_back("18"); graph["17"].push_back("23");

    graph.insert(make_pair("18", vector<string>()));
    graph["18"].push_back("12"); graph["18"].push_back("17"); graph["18"].push_back("24");

    graph.insert(make_pair("19", vector<string>()));
    graph["19"].push_back("13"); graph["19"].push_back("20"); graph["19"].push_back("25");

    graph.insert(make_pair("20", vector<string>()));
    graph["20"].push_back("14"); graph["20"].push_back("19"); graph["20"].push_back("21"); graph["20"].push_back("26");

    graph.insert(make_pair("21", vector<string>()));
    graph["21"].push_back("15"); graph["21"].push_back("20"); graph["21"].push_back("22"); graph["21"].push_back("27");

    graph.insert(make_pair("22", vector<string>()));
    graph["22"].push_back("16"); graph["22"].push_back("21"); graph["22"].push_back("23"); graph["22"].push_back("28");

    graph.insert(make_pair("23", vector<string>()));
    graph["23"].push_back("17"); graph["23"].push_back("22"); graph["23"].push_back("24"); graph["23"].push_back("29");

    graph.insert(make_pair("24", vector<string>()));
    graph["24"].push_back("18"); graph["24"].push_back("23"); graph["24"].push_back("30");

    graph.insert(make_pair("25", vector<string>()));
    graph["25"].push_back("19"); graph["25"].push_back("26"); graph["25"].push_back("31");

    graph.insert(make_pair("26", vector<string>()));
    graph["26"].push_back("20"); graph["26"].push_back("25"); graph["26"].push_back("27"); graph["26"].push_back("32");

    graph.insert(make_pair("27", vector<string>()));
    graph["27"].push_back("21"); graph["27"].push_back("26"); graph["27"].push_back("28"); graph["27"].push_back("33");

    graph.insert(make_pair("28", vector<string>()));
    graph["28"].push_back("22"); graph["28"].push_back("27"); graph["28"].push_back("29"); graph["28"].push_back("34");

    graph.insert(make_pair("29", vector<string>()));
    graph["29"].push_back("23"); graph["29"].push_back("28"); graph["29"].push_back("30"); graph["29"].push_back("35");

    graph.insert(make_pair("30", vector<string>()));
    graph["30"].push_back("24"); graph["30"].push_back("29"); graph["30"].push_back("36");

    graph.insert(make_pair("31", vector<string>()));
    graph["31"].push_back("25"); graph["31"].push_back("32");

    graph.insert(make_pair("32", vector<string>()));
    graph["32"].push_back("26"); graph["32"].push_back("31"); graph["32"].push_back("33");

    graph.insert(make_pair("33", vector<string>()));
    graph["33"].push_back("27"); graph["33"].push_back("32"); graph["33"].push_back("34");

    graph.insert(make_pair("34", vector<string>()));
    graph["34"].push_back("28"); graph["34"].push_back("33"); graph["34"].push_back("35");

    graph.insert(make_pair("35", vector<string>()));
    graph["35"].push_back("29"); graph["35"].push_back("34"); graph["35"].push_back("36");

    graph.insert(make_pair("36", vector<string>()));
    graph["36"].push_back("30"); graph["36"].push_back("35");

    return graph;
}

//Breadth First Search Algorithm
vector<string> BFS(map<string, vector<string> > graph, string startingNode, string desiredNode) {
    //Creating containers of visited and queued values
    vector<string> visited;
    queue<vector<string> > queue;
    
    queue.push(vector<string>(1, startingNode)); //starting the queue with our starting node
    
    while (!queue.empty()) {
        vector<string> path = queue.front(); //first item in queue
        queue.pop(); //removes first item

        string currNode = path.back(); //tailed node becomes current node

        //iterates through a set to find the current value in the path
        vector<string>::iterator findCurr = find(visited.begin(), visited.end(), currNode);

        if (findCurr == visited.end()) {
            visited.push_back(currNode); //marks as visited

            vector<string> adjNodes = graph[currNode]; //nearby nodes of current

            for (vector<string>::iterator it = adjNodes.begin(); it != adjNodes.end(); it++) { //iterates the adjNodes
                vector<string> newPath = path;
                newPath.push_back(*it); //add neighbor to newPath vector
                queue.push(newPath); //adds resultant path to queue

                if (*it == desiredNode) { //if desired found, return current path
                    return newPath;
                }
            }
        }
    }
    return vector<string>();
}

//Remove edges of wanted nodes
map<string, vector<string> > removeEdges(map<string, vector<string> > graph, vector<string> removeVec) {
    for (vector<string>::iterator it = removeVec.begin(); it != removeVec.end(); it++) { //for every edges of the current node...
        vector<string> adjNodes = graph[*it];
        for (vector<string>::iterator edges = adjNodes.begin(); edges != adjNodes.end(); edges++) { //for every node of those edges...

            //disconnects the connected nodes from current edge
            vector<string>::iterator removeNode = remove(graph[*edges].begin(), graph[*edges].end(), *it);

            graph[*edges].erase(removeNode, graph[*edges].end()); //getting rid of edges
        }
        graph[*it].clear(); //removing current node
    }
    return graph;
}

//Print the path and movements for a robot to move from start to desired node
void roboMove(map<string, vector<string> > graph, string startNode, string desiredNode) {
    vector<string> roboPath = BFS(graph, startNode, desiredNode); //can be DFS as well
 
    //Movement of Robotino based on index
    for (int i = 0; i < int(carPath.size()) - 1; i++) {
 
        //Position is based on current and next value in path
        int currPos = atoi(carPath.at(i).c_str());
        int nextPos = atoi(carPath.at(i + 1).c_str());
 
        //values being +/- are based on your grid setup
        if ((currPos + 6) == nextPos) {
            moveY(0,1); //up
        }
 
        if ((currPos - 6) == nextPos) {
            moveY(1,0); //down
        }
 
        if ((currPos - 1) == nextPos) {
            moveX(1,0); //left
        }
 
        if ((currPos + 1) == nextPos) {
            moveX(0,1); //right
        }
    }
    omni.setVelocity(0,0,0); //precautionary sake
}

int main(int argc, char **argv) {
    MyCom com;

    //initialize the actors
    if(argc > 1){
        com.setAddress(argv[1]);
        cout << "Trying to connect to " << argv[1] << "..." << endl;
    }else {
        com.setAddress("172.26.201.2");
        cout << "Trying to connect..." << endl;
    }
    try{
        com.connectToServer(true);
        cout << "Successfully connected!" << endl;
    } catch(...) {
        cout << "Unable to connect! Program stopped." << endl;
        return 0;
    }

    //setting up graph
    map<string, vector<string> > graph = graphFunction();
    
    //Setting variables
    string startingNode = "1";
    string desiredNode = "36";
    int nodeRemoval = 0;
    
    cout << "Starting Node: ";
    cin >> startingNode;
    
    cout << "\nDesired Node: ";
    cin >> desiredNode;

    cout << "\nHow many nodes to remove: ";
    cin >> nodeRemoval;

    //vector that houses the nodes/edges you want to remove from graph
    vector<string> removeVec(nodeRemoval);  
    removeVec.push_back("4");
    removeVec.push_back("8"); 
    removeVec.push_back("9");
    removeVec.push_back("20");
    removeVec.push_back("21");
    removeVec.push_back("28");
    removeVec.push_back("29");
    removeVec.push_back("31");
    string input = "";  
    
    for (int i = 0; i < nodeRemoval; i++) {
	cout << "Node " << i+1 << ": ";
        cin >> input;
        removeVec.push_back(input); 
    }
    
    //Resultant graph after changes
    graph = removeEdges(graph, removeVec);

    roboMove(graph, startingNode, desiredNode);

    return 0;
}
