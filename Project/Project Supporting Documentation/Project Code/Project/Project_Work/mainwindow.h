#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "anonwindow.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_findFile_clicked();

    void on_next_button_clicked();

private:
    Ui::MainWindow *ui;
    AnonWindow *anonWin;
};

#endif // MAINWINDOW_H
