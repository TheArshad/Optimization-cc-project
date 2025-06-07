#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <numeric>

using namespace std;
using ll = long long;

struct Building {
    double x;
    double y;
    double cost;
    int id;
};

struct Point {
    double x;
    double y;
    Point(double xCoordinate = 0, double yCoordinate = 0) : x(xCoordinate), y(yCoordinate) {}
};
//logic
class AxisAlignedPolygonOptimizer {
private:
    vector<Building>& buildingList;
    int numberOfBuildings;
    int minimumRequired;
    double optimalCost;
    vector<Point> optimalPolygon;

    vector<int> selectLowCostBuildingsGreedy() {
        vector<int> selectedBuildingIndices;
        vector<bool> usageFlags(numberOfBuildings, false);

        vector<int> sortedBuildingIndices(numberOfBuildings);
        iota(sortedBuildingIndices.begin(), sortedBuildingIndices.end(), 0);
        sort(sortedBuildingIndices.begin(), sortedBuildingIndices.end(), [&](int first, int second) {
            return buildingList[first].cost < buildingList[second].cost;
        });

        for (int index = 0; index < minimumRequired && index < numberOfBuildings; ++index) {
            selectedBuildingIndices.push_back(sortedBuildingIndices[index]);
            usageFlags[sortedBuildingIndices[index]] = true;
        }
        for (int index = minimumRequired; index < numberOfBuildings && index < minimumRequired + 10; ++index) {
            int buildingIndex = sortedBuildingIndices[index];
            if (buildingList[buildingIndex].cost < 0) {
                vector<int> trialSelection = selectedBuildingIndices;
                trialSelection.push_back(buildingIndex);

                double currentPerimeter = calculateBoundingPerimeter(selectedBuildingIndices);
                for (int idx : selectedBuildingIndices) currentPerimeter += buildingList[idx].cost;

                double trialPerimeter = calculateBoundingPerimeter(trialSelection);
                for (int idx : trialSelection) trialPerimeter += buildingList[idx].cost;

                if (trialPerimeter < currentPerimeter) {
                    selectedBuildingIndices = move(trialSelection);
                }
            }
        }
        return selectedBuildingIndices;
    }

    vector<int> selectBuildingsWithNegativeCost() {
        vector<int> negativeBuildingIndices;
        for (int idx = 0; idx < numberOfBuildings; ++idx) {
            if (buildingList[idx].cost < 0) {
                negativeBuildingIndices.push_back(idx);
            }
        }
        if (negativeBuildingIndices.size() >= minimumRequired) {
            return negativeBuildingIndices;
        }

        vector<int> positiveBuildingIndices;
        for (int idx = 0; idx < numberOfBuildings; ++idx) {
            if (buildingList[idx].cost >= 0) {
                positiveBuildingIndices.push_back(idx);
            }
        }
        sort(positiveBuildingIndices.begin(), positiveBuildingIndices.end(), [&](int first, int second) {
            return buildingList[first].cost < buildingList[second].cost;
        });

        vector<int> combinedSelection = negativeBuildingIndices;
        for (int posIndex : positiveBuildingIndices) {
            if (combinedSelection.size() >= minimumRequired) break;
            combinedSelection.push_back(posIndex);
        }
        return combinedSelection;
    }

    double calculateBoundingPerimeter(const vector<int>& buildingIndices) {
        if (buildingIndices.empty()) return 0;

        double minX = buildingList[buildingIndices[0]].x;
        double maxX = buildingList[buildingIndices[0]].x;
        double minY = buildingList[buildingIndices[0]].y;
        double maxY = buildingList[buildingIndices[0]].y;
        for (int idx : buildingIndices) {
            minX = min(minX, buildingList[idx].x);
            maxX = max(maxX, buildingList[idx].x);
            minY = min(minY, buildingList[idx].y);
            maxY = max(maxY, buildingList[idx].y);
        }
        return 2 * ((maxX - minX) + (maxY - minY));
    }

    vector<Point> constructBoundingRectangle(const vector<int>& buildingIndices) {
        if (buildingIndices.empty()) return {};

        double minX = buildingList[buildingIndices[0]].x;
        double maxX = buildingList[buildingIndices[0]].x;
        double minY = buildingList[buildingIndices[0]].y;
        double maxY = buildingList[buildingIndices[0]].y;
        for (int idx : buildingIndices) {
            minX = min(minX, buildingList[idx].x);
            maxX = max(maxX, buildingList[idx].x);
            minY = min(minY, buildingList[idx].y);
            maxY = max(maxY, buildingList[idx].y);
        }
        return { Point(minX, minY), Point(maxX, minY), Point(maxX, maxY), Point(minX, maxY) };
    }

    void evaluateAndUpdateOptimalSolution(const vector<int>& candidateBuildingIndices) {
        if (candidateBuildingIndices.size() < minimumRequired) return;

        double boundingPerimeter = calculateBoundingPerimeter(candidateBuildingIndices);
        double totalBuildingCost = 0;
        for (int idx : candidateBuildingIndices) {
            totalBuildingCost += buildingList[idx].cost;
        }
        double combinedCost = boundingPerimeter + totalBuildingCost;

        if (combinedCost < optimalCost) {
            optimalCost = combinedCost;
            optimalPolygon = constructBoundingRectangle(candidateBuildingIndices);
        }
    }

public:
    AxisAlignedPolygonOptimizer(vector<Building>& buildings, int k)
        : buildingList(buildings), numberOfBuildings(buildings.size()), minimumRequired(k), optimalCost(1e18) {}

    pair<double, vector<Point>> findMinimumCostPolygon() {
        evaluateAndUpdateOptimalSolution(selectLowCostBuildingsGreedy());
        evaluateAndUpdateOptimalSolution(selectBuildingsWithNegativeCost());

        vector<int> allBuildingIndices(numberOfBuildings);
        iota(allBuildingIndices.begin(), allBuildingIndices.end(), 0);
        sort(allBuildingIndices.begin(), allBuildingIndices.end(), [&](int first, int second) {
            return buildingList[first].cost < buildingList[second].cost;
        });
        vector<int> firstKBuildings(allBuildingIndices.begin(), allBuildingIndices.begin() + minimumRequired);
        evaluateAndUpdateOptimalSolution(firstKBuildings);

        return { optimalCost, optimalPolygon };
    }
};

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int numberOfBuildings, minimumRequired;
    cin >> numberOfBuildings >> minimumRequired;

    vector<Building> buildingList(numberOfBuildings);
    for (int i = 0; i < numberOfBuildings; ++i) {
        cin >> buildingList[i].x >> buildingList[i].y >> buildingList[i].cost;
    }

    AxisAlignedPolygonOptimizer optimizer(buildingList, minimumRequired);
    auto [finalCost, optimalPolygon] = optimizer.findMinimumCostPolygon();

    cout << fixed << setprecision(6) << finalCost << '\n';
    if (!optimalPolygon.empty()) {
        int vertexCount = optimalPolygon.size();
        for (int i = 0; i < vertexCount; ++i) {
            int nextIndex = (i + 1) % vertexCount;
            cout << fixed << setprecision(6)
                 << optimalPolygon[i].x << ' ' << optimalPolygon[i].y << ' '
                 << optimalPolygon[nextIndex].x << ' ' << optimalPolygon[nextIndex].y << '\n';
        }
    }

    return 0;
}
