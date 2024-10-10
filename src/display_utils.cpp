//
// Created by davide on 28/09/23.
//
#include "display_utils.h"

void showProgressBar(int current, int total, int bar_width) {
    float progress = static_cast<float>(current)/total;
    int bar_length = static_cast<int>(progress*bar_width);
    std::cout << "[";
    for (int i = 0; i < bar_width; i++){
        if (i < bar_length)
            std::cout << "=";
        else
            std::cout << " ";
    }
    std::cout << "]" << int(progress * 100.0) << "%\r";
    std::cout.flush();
}
