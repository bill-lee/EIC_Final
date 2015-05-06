#include "eic_test.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    EIC_Test w;
    w.show();    
    return a.exec();
}
