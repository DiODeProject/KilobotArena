#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
#if QT_VERSION >= QT_VERSION_CHECK(5, 2, 0)
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps,true);
#endif
    return a.exec();
}
