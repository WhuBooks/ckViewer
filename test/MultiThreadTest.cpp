//
// Created by books on 2018/5/6.
//

#include <iostream>
#include <algorithm>


#include <CloudHandler.h>

int main()
{
    Viewer::CloudHandler cloud_handler;

    for(int i=0;i<10;i++)
    {
        cloud_handler.Add(std::to_string(i),i,i,i);
    }

    for(int i=0;i<10;i++)
    {
        std::thread tmp([&](){
            int index=i;
            while(true)
            {
                std::cout<<"Current Index ~ "<<index<<std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        });

        tmp.detach();
    }

    int ind=0;
    while(ind<1000)
    {
        ind++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 1;

}