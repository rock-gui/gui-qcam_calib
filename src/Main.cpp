#include <stdio.h>
#include "QCamCalib.hpp"

int main( int argc, char* argv[] )
{
    QApplication a(argc, argv);
    QCamCalib w;
    w.show();
    return a.exec();
}
