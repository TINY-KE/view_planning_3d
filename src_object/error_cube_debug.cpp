#include <iostream>
#include <cmath>
#include <vector>
#include "MapObject.h"

int main(){
    // SdfObject ob( 3, 0, 0.5, 
    //             2.5, 2, 1, 
    //             0, 0, 0);
    double x=3, y=0, z=0.5;
    double lenth=2.5, width=2, height=1;
    double yaw = 0;
    MapObject* ob = new MapObject();
    ob->Update_Twobj(x,y,z,yaw);
    ob->Update_object_size(lenth,width,height);


    return 0;
}