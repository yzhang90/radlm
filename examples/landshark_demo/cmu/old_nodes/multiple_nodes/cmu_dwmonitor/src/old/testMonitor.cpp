/*
 *      testMonitor.cpp
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */

#include <iostream>
extern "C"
{
#include "dwmonitor.h"
}

using namespace std;

int main()
{
    double D[3] = {1, 2, 3};
    double X[5] = {1, 2, 3, 4, 5};
    cout << "\nHmm" << endl;
    int result = dwmonitor(X, D);
    cout << "\n" << result << endl;
    cout << "\nSuccess BRUV";
    cout << endl;
    return 0;
}
