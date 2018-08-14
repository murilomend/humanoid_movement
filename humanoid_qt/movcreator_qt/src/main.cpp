#include <QApplication>
#include "../include/movcreator_qt/movecreator.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    movcreator_qt::MoveCreator moveCreator(argc, argv);
    moveCreator.show();

    return a.exec();
}
