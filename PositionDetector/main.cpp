#include <iostream>
#include "localizer.h"

using namespace std;

int main(int argc, char* argv[]) {

    localizer posFinder;

    posFinder.init();

    while(posFinder.getFrame()) {

        posFinder.showResult();

        char c = waitKey(15);
        if( c == 27 )
            break;
    }

    return 0;
}
