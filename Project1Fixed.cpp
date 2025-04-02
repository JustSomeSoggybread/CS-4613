/*
  Project1.cpp
  Evan Lee and Seha Kim
  Simulate a robot navigating a graph
  November/08/2024
*/

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <queue>
#include <tuple>
#include <set>
#include <algorithm>
#include <iomanip>
#include <unordered_map>
#include <string>


using namespace std;


/**
 * @brief               Constant variables
 * @param INPUTFILE     name of input file
 * @param OUTPUTFILE    name of output file
 * @param ROWCOUNT      number of rows (height/y of graph)
 * @param COLCOUNT      number of columns (width/x of graph)
 * @param k             "turning" weight / angle change penalty
 * @param MOVES         vector of vectors in the form {move identifier, x transformation, y transformation}
 * @param map           future map of height ROWCOUNT and width COLCOUNT, initially a set of 0s
*/ 
const string INPUTFILE = "input1.txt";
const string OUTPUTFILE = "output1_k4.txt";
const int ROWCOUNT = 30;
const int COLCOUNT = 50;
const int k = 4;
const vector<vector<int>> MOVES = {{0, 1, 0}, {1, 1, 1}, {2, 0, 1}, {3, -1, 1}, {4, -1, 0}, 
                                    {5, -1, -1}, {6, 0, -1}, {7, 1, -1}};
vector<vector<int>> map(ROWCOUNT, vector<int>(COLCOUNT));


/**
 * @brief Node class. Used while A* searching
 */

struct Node {
    /**
     * @brief       constructs a Node with given parameters
     * @param i_x   x position of node
     * @param i_y   y position of node
     * @param pCost path cost of node
     * @param hCost heuristic code of node
     * @param pNode parent node
     * @param nMove last move to reach current node (used in calculating "turning" cost)
     */
    Node(int i_x, int i_y, double pCost, double hCost, Node* pNode = nullptr, int nMove = -1)
        : x(i_x), y(i_y), pathCost(pCost), heuristic(hCost), totalCost(pCost + hCost), parent(pNode), moveTo(nMove){}

    int x;
    int y;
    double pathCost;
    double heuristic;
    double totalCost;
    Node* parent;
    int moveTo;

    // overload < operator to allow for node comparison during priority queue in A* search
    bool operator<(const Node& other) const{
        return totalCost > other.totalCost;
    }
};


/**
 * @brief       calculate the heuristic cost from a given x,y to a goal x, y
 * @param n_x   node's x coordinate
 * @param n_y   node's y coordinate
 * @param g_x   goal's x coordinate
 * @param g_y   goal's y coordinate
 */
double calcHeuristic(int n_x, int n_y, int g_x, int g_y){
    return sqrt(pow(n_x - g_x, 2) + pow(n_y - g_y, 2));
}


/**
 * @brief           calculate the cost the action cost to move between two given coordinates
 * @param n_x       initial x coordinate
 * @param n_y       initial y coordinate
 * @param g_x       goal x coordinate
 * @param g_y       goal y coordinate
 * @param oldFace   where the robot was "facing" on its initial coordinate
 * @param newFace   where the robot must "face" to move to its goal coordinate
 * @param k         angle change penalty
 */
double calcMoveCost(int n_x, int n_y, int g_x, int g_y, int oldFace, int newFace, double k){
    double turnCost = 0.0;
    if (oldFace != -1){
        // turnCost = minimum number of 45 degree turns (int jumps) the robot must make to face its new direction.
        int jumps = ((newFace - oldFace + 8)% 8 > 4) ? (8 - ((newFace - oldFace + 8)% 8)) : ((newFace - oldFace + 8)% 8);
        turnCost = k * (jumps) / 4;
    } 
    double moveCost = (newFace % 2 == 0) ? 1 : sqrt(2);
    return turnCost + moveCost;
}


/**
 * @brief           performs an A* search along a given graph
 * @param start_x   starting x coordinate of robot
 * @param start_y   starting y coordinate of robot
 * @param goal_x    goal x coordinate of robot
 * @param goal_y    goal y coordinate of robot
 * @param map       graph along which robot moves
 * @param weight    angle change penalty
 * @return          returns a tuple in the form <tree depth, number of nodes generated, 
 *                                              list of moves in the found solution, 
 *                                              f(n) values of nodes along the solution path>
 */
tuple<int, int, vector<int>, vector<double>> search(int start_x, int start_y, int goal_x, int goal_y, vector<vector<int>>& map, double weight) {
   priority_queue<Node> frontier;    // Priority queue for open nodes
   unordered_map<int, bool> visited;    // Set to keep track of visited nodes
   int nodeCount = 1;


   Node* start_node = new Node(start_x, start_y, 0, calcHeuristic(start_x, start_y, goal_x, goal_y), nullptr, -1);
   frontier.push(*start_node);


   while (!frontier.empty()) {
       Node cur = frontier.top();
       frontier.pop();

        // checks if robot has reached goal position
       if ((cur.x == goal_x) && (cur.y == goal_y)) {
            vector<int> solution;
            vector<double> costs;
            int depth = 0;

            //trace back through solution path
            Node* checkNode = &cur;
            while (checkNode) {
                if (checkNode->moveTo != -1){
                    solution.push_back(checkNode->moveTo);
                }
                costs.push_back(checkNode->totalCost);
                checkNode = checkNode->parent;
                depth++;
            }

            reverse(solution.begin(), solution.end());
            reverse(costs.begin(), costs.end());
            delete start_node;
            return{depth, nodeCount, solution, costs};

       }

        // otherwise, expand possible children
        pair<int, int> position;
        position.first = cur.x;
        position.second = cur.y;
        visited[cur.x + (COLCOUNT * cur.y)] = true;

        // for each possible move, check if a child is possible. If it is, add the child node to the frontier
        for (vector<int> move : MOVES) {
            int ni = cur.x + move[1], nj = cur.y + move[2];
            if (0 <= nj && nj < ROWCOUNT && 0 <= ni && ni < COLCOUNT && (map[ROWCOUNT - nj - 1][ni] != 1)) {
                if (!visited[ni + (COLCOUNT * nj)]){
                    Node* childNode = new Node(ni, nj, 
                        cur.pathCost + calcMoveCost(cur.x, cur.y, ni, nj, cur.moveTo, move[0], k), 
                        calcHeuristic(ni, nj, goal_x, goal_y), 
                        new Node(cur), move[0]);
                frontier.push(*childNode);
                nodeCount++;
                }
            }
        }
   }

   delete start_node;
   return {0, nodeCount, {}, {}};
}

/**
 * @brief   model A* search along a graph
 * @return  0 on success. Prints output to specified text file
 */
int main() {

   ifstream inputFile(INPUTFILE);
   ofstream outputFile(OUTPUTFILE);

   int start_x, start_y, goal_x, goal_y;
   inputFile >> start_x >> start_y >> goal_x >> goal_y;


   // Copy graph to map
   vector<vector<int>> map(ROWCOUNT, vector<int>(COLCOUNT));
   for (int i = 0; i < ROWCOUNT; ++i) {
       for (int j = 0; j < COLCOUNT; ++j) {
           inputFile >> map[i][j];
       }
   }

    int depth, nodes_generated;
    vector<int> solution;
    vector<double>costs;
    // Initialize A* search
    tie(depth, nodes_generated, solution, costs) = search(start_x, start_y, goal_x, goal_y, map, k);

    // Print solution to output file
    if (!solution.empty()) {
            int cur_x = start_x;
            int cur_y = start_y;
        for (int move : solution) {
            int next_x = cur_x, next_y = cur_y;
            next_x += MOVES[move][1];
            next_y += MOVES[move][2];
            if (map[ROWCOUNT - next_y - 1][next_x] == 0){
                    map[ROWCOUNT - next_y - 1][next_x] = 4;
            }
            cur_x = next_x;
            cur_y = next_y;
        }

        outputFile << fixed << setprecision(1);

        outputFile << depth << "\n" << nodes_generated << "\n";

        for (int move : solution) outputFile << move << " ";
        outputFile << "\n";

        for (double cost : costs) outputFile << cost << " ";
        outputFile << "\n";

        for (vector<int> row : map) {
            for (int cell : row) outputFile << cell << " ";
            outputFile << "\n";
        }

        cout << "Successfully output to " << OUTPUTFILE;
    } else {
        cout << "No solution available.\n";
    }

   return 0;
}


