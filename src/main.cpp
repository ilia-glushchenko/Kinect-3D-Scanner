#include <QtWidgets/QApplication>
#include <gui/scannerwidget.h>

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    ScannerWidget scanerWidget;
    scanerWidget.show();
    return a.exec();
}
