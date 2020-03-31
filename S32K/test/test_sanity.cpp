/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 */
#include <iostream>

#include "libuavcan/media/S32K/s32k_libuavcan.hpp"

int main()
{
    libuavcan::media::S32K_InterfaceManager manager;
    if (manager.getMaxFrameFilters() == 0)
    {
        std::cerr << "InterfaceManager::getMaxFrameFilters() returned 0" << std::endl;
        return -1;
    }
    else
    {
        std::cout << "S32K InterfaceManager appears to compile." << std::endl;
        return 0;
    }
}
