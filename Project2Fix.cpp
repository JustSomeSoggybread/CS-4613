/*
  Project1.cpp
  Evan Lee and Seha Kim
  Solve a Kropki Sudoku puzzle
  December/12/2024
*/

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cstring>
#include <string>

using namespace std;

/**
 * @brief           Constant variables
 * @param SIZE      Size of Sudoku puzzle (number of rows / columns)
 * @param INPUTFILE The name of the file to be read from
 * @param OUTPUFILE The name of the file to be written to
 */
const int SIZE = 9;
const string INPUTFILE = "Input2.txt";
const string OUTPUTFILE = "Output2.txt";

/**
 * @brief Cell class. Used to represent Sudoku cells
 */
struct Cell {
    // Cells have a default value of 0 and a possible domain of all numbers from 1 through 9
    int value = 0;
    vector<int> d{1,2,3,4,5,6,7,8,9};
};

/**
 * @param board         A representation of the Sudoku puzzle. Represented as a 2-dimensional array of cells.
 * @param hConstraints  A representation of the horizontal constraints of the Sudoku puzzle. Represented as a 2-dimensional array of integers.
 * @param vConstraints  A representation of the vertical constraints of the Sudoku puzzle. Represented as a 2-dimensional array of integers.
 */
Cell board[SIZE][SIZE];
int hConstraints[SIZE][SIZE-1];
int vConstraints[SIZE-1][SIZE];

/**
 * @brief               Check if a given value for a cell will violate a rule. If there is no violation, assigns that value to the cell
 * @param row           The row of the cell
 * @param col           The column of the cell
 * @param val           The value to be given.
 * @return              Returns true if the given value does not violate any rules.
 *                      Returns false if the given value violates a rules.
 */
bool validate(int row, int col, int val){
    // Check for a digit overlap within the cell's 3x3 neighborhood
    int startRow = row - (row % 3), startCol = col - (col % 3);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (board[startRow + i][startCol + j].value == val) {return false;}
        }
    }

    // Check for a digit overlap within the cell's rows and columns
    for (int i = 0; i < 9; ++i) {
        if (board[row][i].value == val || board[i][col].value == val) {return false;}
    }
    
    // Check for heuristic violations between the cell and its neighbor above it
    if (row > 0) {
        int above = board[row - 1][col].value;
        if (above != 0){
            if (vConstraints[row - 1][col] == 1 && abs(above - val) != 1) {return false;}
            if (vConstraints[row - 1][col] == 2 && (above != 2 * val && val != 2 * above)) {return false;}
        }

    }

    // Check for heuristic violations between the cell and its neighbor below it
    if (row < SIZE - 1) {
        int below = board[row + 1][col].value;
        if (below != 0){
            if (vConstraints[row][col] == 1 && abs(below - val) != 1) {return false;}
            if (vConstraints[row][col] == 2 && (below != 2 * val && val != 2 * below)) {return false;}
        }
    }

    // Check for heuristic violations between the cell and its neighbor to the left
    if (col > 0) {
        int left = board[row][col - 1].value;
        if (left != 0){
            if (hConstraints[row][col - 1] == 1 && abs(left - val) != 1) {return false;}
            if (hConstraints[row][col - 1]== 2 && (left != 2 * val && val != 2 * left)) {return false;}
        }
    }

    // Check for heuristic violations between the cell and its neighbor to the right.
    if (col < SIZE - 1) {
        int right = board[row][col + 1].value;
        if (right != 0){
            if (hConstraints[row][col] == 1 && abs(right - val) != 1) {return false;}
            if (hConstraints[row][col] == 2 && (right != 2 * val && val != 2 * right)) {return false;}
        }
    }
    
    return true;
}

/**
 * @brief       Uses minimum remaining value and degree heuristics to find the next unassigned variable to assign a value
 * @return      A pair of integers, denoting the row and column of the next cell to assign a value
 */
pair<int, int> find_next() {
    int min_remaining = 10;
    int max_degree = -1;
    pair<int, int> selected = {-1, -1};

    for (int row = 0; row < SIZE; row++) {
        for (int col = 0; col < SIZE; col++) {
            if (board[row][col].value == 0) { 
                int remaining = board[row][col].d.size();
                
                // Check for the degree of the cell
                int degree = 0;

                // Increment degree for each cell filled in the cell's 3x3 neighborhood
                int box_row = row - row % 3, box_col = col - col % 3;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        if (box_row + i != row && box_col + j != col && board[box_row + i][box_col + j].value == 0) {degree++;}
                    }
                }

                // Increment degree for each cell filled in the cell's row and column
                for (int i = 0; i < SIZE; i++) {
                    if (i != col && board[row][i].value == 0) {degree++;}
                    if (i != row && board[i][col].value == 0) {degree++;}
                }

                    // Check for heuristic violations between the cell and its neighbor above it
                if (row > 0) {
                    if (vConstraints[row - 1][col] == 1) {degree++;}
                    if (vConstraints[row - 1][col] == 2) {degree++;}
                }

                // Check for heuristic violations between the cell and its neighbor below it
                if (row < SIZE - 1) {
                    if (vConstraints[row][col] == 1) {degree++;}
                    if (vConstraints[row][col] == 2) {degree++;}

                }

                // Check for heuristic violations between the cell and its neighbor to the left
                if (col > 0) {
                    if (hConstraints[row][col - 1] == 1) {degree++;}
                    if (hConstraints[row][col - 1]== 2) {degree++;}
                }

                // Check for heuristic violations between the cell and its neighbor to the right.
                if (col < SIZE - 1) {
                        if (hConstraints[row][col] == 1) {degree++;}
                        if (hConstraints[row][col] == 2) {degree++;}
                }

                // This cell takes precedence if it has the smallest domain of remaining cells or is tied
                // for least remaining possible values and has the highest degree
                if (remaining < min_remaining || (remaining == min_remaining && degree > max_degree)) {
                    selected = {row, col};
                    min_remaining = remaining;
                    max_degree = degree;
                }
            }
        }
    }

    return selected;
}

/**
 * @brief       Forward checking - checks if implementing a value will empty
 *              the domain of an unassigned neighbor.
 * @param row   The row of the cell to be checked
 * @param col   The column of the cell to be checked
 * @param val   The value to be implemented
 * @return      True if implementing the value does NOT result in an unassigned
 *              neighbor with an empty domain.
 *              False if implementing the value DOES result in an unassigned
 *              neighbor with an empty domain.
*/ 
bool forward_check(int row, int col, int val) {

    // Forward check columns and rows
    for (int i = 0; i < SIZE; i++) {
        if (i != col && board[row][i].value == 0) {
            vector<int>& domain = board[row][i].d;
            domain.erase(remove(domain.begin(), domain.end(), val), domain.end());
            if (domain.empty()) {return false;}
        }
        if (i != row && board[i][col].value == 0) {
            vector<int>& domain = board[i][col].d;
            domain.erase(remove(domain.begin(), domain.end(), val), domain.end());
            if (domain.empty()) {return false;}
        }
    }

    // Forward check within 3x3 neighborhood
    int box_row = row - row % 3, box_col = col - col % 3;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (box_row + i != row && box_col + j != col && board[box_row + i][box_col + j].value == 0) {
                vector<int>& domain = board[box_row + i][box_col + j].d;
                domain.erase(remove(domain.begin(), domain.end(), val), domain.end());
                if (domain.empty()) {return false;}
            }
        }
    }
    return true;
}

/**
 * @brief   backtracking search for the puzzle
 * @return  true if every empty cell has been filled out (success)
 *          false if a cell cannot be filled out (failure)
 */
bool backtracking_search() {
    // find next unassigned cell
    pair<int, int> coord = find_next(); 
    // return success if no unassigned cells left
    if (coord.first == -1 && coord.second == -1) return true;

    // otherwise, begin testing values within that cell's domain
    for (int val : board[coord.first][coord.second].d) {
        // Validate domain values
        if (validate(coord.first, coord.second, val)) { 
            // Create a list of affected neighbors
            vector<pair<int, int>> neighbors;
            // Forward check for if validated value causes domain violations
            if (forward_check(coord.first, coord.second, val)){                

                // Assigns the value to the cell if no heuristics are violated
                board[coord.first][coord.second].value = val; 

                // Add affected neighbors in 3x3 neighborhood to list
                int box_row = coord.first - coord.first % 3;
                int box_col = coord.second - coord.second % 3;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        if (box_row + i != coord.first && box_col + j != coord.second && board[box_row + i][box_col + j].value == 0) {
                            neighbors.emplace_back(box_row + i, box_col + j);
                        }
                    }
                }

                // Add affected neighbors in column / row to list
                for (int i = 0; i < SIZE; i++) {
                    if (i != coord.first && board[i][coord.second].value == 0) {neighbors.emplace_back(i, coord.second);}
                    if (i != coord.second && board[coord.first][i].value == 0) {neighbors.emplace_back(coord.first, i);}
                }
            
                if (backtracking_search()) {return true;}
            }

            // If forward check or recursive search fails, undo assignment 
            // and reset the domains of the cell's neighbors
            board[coord.first][coord.second].value = 0;
            for (const pair<int, int>& cell : neighbors) {
                board[cell.first][cell.second].d.push_back(val);
            }
        }
    }

    return false;
}

/**
 * @brief       sudoku solver
 * @return      Prints output to specified text file
 */
int main() {
    string inputFile = INPUTFILE; // Replace with actual input file
    string outputFile = OUTPUTFILE;

    ifstream input(inputFile);

    // copy game board to board
    for (int i = 0; i < SIZE; i++){
        for (int j = 0; j < SIZE; j++){
            input >> board[i][j].value;
            if (board[i][j].value != 0){
                board[i][j].d.clear();
                board[i][j].d.push_back(board[i][j].value);
            }
        }
    }
    input.ignore();
    
    // copy horizontal constraints graph
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE-1; j++) {
            input >> hConstraints[i][j];
        }
    }
    input.ignore();

    // copy vertical constraints graph
    for (int i = 0; i < SIZE-1; i++) {
        for (int j = 0; j < SIZE; j++) {
            input >> vConstraints[i][j];
        }
    }
    input.close();


    // Initialize backtracking search. Output if a solution is found.
    if (backtracking_search()){
        cout << "Solved. Output to " << outputFile << endl;

        ofstream output(outputFile);
        for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            output << board[i][j].value << " ";
        }
        output << "\n";
    }
    output.close();
    }

    cout << "Program Finished";

}
