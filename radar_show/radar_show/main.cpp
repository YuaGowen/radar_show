#include "mainwindow.h"

#include <QApplication>
#include <QStyleFactory>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    /* 高分辨率缩放自适应 */
    a.setAttribute(Qt::AA_EnableHighDpiScaling);

    /* 整体风格扁平化 */
    a.setStyle(QStyleFactory::create("Fusion"));
    MainWindow w;
    w.show();
    return a.exec();
}
