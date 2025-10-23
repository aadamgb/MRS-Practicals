#include <iostream>
#include <vector>
#include <iomanip>
#include <cstdlib>
#include <ctime>

// Function to print a 2D vector
void printMatrix(const std::vector<std::vector<double>>& matrix, const std::string& name) {
    std::cout << name << ":\n";
    for (const auto& row : matrix) {
        for (double val : row) {
            std::cout << std::setw(8) << std::fixed << std::setprecision(2) << val << " ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

// Function to print a 1D vector
void printVector(const std::vector<double>& vec, const std::string& name) {
    std::cout << name << ": ";
    for (double val : vec) {
        std::cout << std::setw(8) << std::fixed << std::setprecision(2) << val << " ";
    }
    std::cout << "\n\n";
}

// Your original function
std::vector<double> findMinRowValues(const std::vector<std::vector<double>>& cost, int n_uavs) {
    std::vector<double> min_row(n_uavs, cost[0][0]);
    
    for (int i = 0; i < n_uavs; i++) {
        for (int j = 1; j < n_uavs; j++) {
            double new_min = cost[i][j];
            if (new_min < min_row[i]) {
                min_row[i] = new_min;
            }
        }
    }
    
    return min_row;
}

// Alternative implementation for comparison
std::vector<double> findMinRowValuesAlternative(const std::vector<std::vector<double>>& cost, int n_uavs) {
    std::vector<double> min_row(n_uavs);
    
    for (int i = 0; i < n_uavs; i++) {
        min_row[i] = cost[i][0]; // Initialize with first element of the row
        for (int j = 1; j < n_uavs; j++) {
            if (cost[i][j] < min_row[i]) {
                min_row[i] = cost[i][j];
            }
        }
    }
    
    return min_row;
}

// Test function
void testMinRowFunction() {
    std::cout << "=== Testing Min Row Function ===\n\n";
    
    // Test Case 1: Small matrix
    std::cout << "Test Case 1: 3x3 Matrix\n";
    int n_uavs1 = 3;
    std::vector<std::vector<double>> cost1 = {
        {5.0, 2.0, 8.0},
        {1.0, 9.0, 4.0},
        {7.0, 3.0, 6.0}
    };
    
    printMatrix(cost1, "Cost Matrix");
    std::vector<double> result1 = findMinRowValues(cost1, n_uavs1);
    printVector(result1, "Min Row Values");
    
    // Expected: [2.0, 1.0, 3.0]
    
    // Test Case 2: Matrix with negative values
    std::cout << "Test Case 2: Matrix with Negative Values\n";
    int n_uavs2 = 4;
    std::vector<std::vector<double>> cost2 = {
        {10.5, -2.3, 8.7, 5.1},
        {-1.2, 15.6, 3.4, 9.8},
        {7.9, 2.1, -5.6, 12.3},
        {4.4, 8.8, 6.6, 1.1}
    };
    
    printMatrix(cost2, "Cost Matrix");
    std::vector<double> result2 = findMinRowValues(cost2, n_uavs2);
    printVector(result2, "Min Row Values");
    
    // Test Case 3: Random matrix
    std::cout << "Test Case 3: Random 5x5 Matrix\n";
    int n_uavs3 = 5;
    std::vector<std::vector<double>> cost3(n_uavs3, std::vector<double>(n_uavs3));
    
    srand(time(0));
    for (int i = 0; i < n_uavs3; i++) {
        for (int j = 0; j < n_uavs3; j++) {
            cost3[i][j] = (rand() % 1000) / 10.0; // Random values between 0 and 99.9
        }
    }
    
    printMatrix(cost3, "Cost Matrix");
    std::vector<double> result3 = findMinRowValues(cost3, n_uavs3);
    printVector(result3, "Min Row Values");
    
    // Compare with alternative implementation
    std::vector<double> result3_alt = findMinRowValuesAlternative(cost3, n_uavs3);
    printVector(result3_alt, "Alternative Implementation");
    
    // Test Case 4: Edge case - all same values
    std::cout << "Test Case 4: All Same Values\n";
    int n_uavs4 = 3;
    std::vector<std::vector<double>> cost4(n_uavs4, std::vector<double>(n_uavs4, 7.5));
    
    printMatrix(cost4, "Cost Matrix");
    std::vector<double> result4 = findMinRowValues(cost4, n_uavs4);
    printVector(result4, "Min Row Values");
}

int main() {
    testMinRowFunction();
    
    // Additional verification
    std::cout << "=== Manual Verification Example ===\n";
    std::vector<std::vector<double>> manual_test = {
        {10.0, 20.0, 5.0},
        {15.0, 3.0, 25.0},
        {8.0, 12.0, 18.0}
    };
    
    printMatrix(manual_test, "Manual Test Matrix");
    std::vector<double> manual_result = findMinRowValues(manual_test, 3);
    printVector(manual_result, "Computed Min Values");
    
    std::cout << "Expected: [5.0, 3.0, 8.0]\n";
    
    return 0;
}