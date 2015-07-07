/*
 *      testingObstacles.cpp
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */

#include "obstacles.h"
#include <string>
using namespace std;
int main()
{
    environmentObstacles o;
    cout << "Obstacle Count: " << o.obsCount;
    cout << endl;
    o.initializeObstacleTag(1, 1, 1, "fatma");
    cout << "Obstacle Count: " << o.obsCount;
    cout << endl;
    return 0;
}

