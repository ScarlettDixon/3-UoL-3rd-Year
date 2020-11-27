#ifndef ANONWINDOW_H
#define ANONWINDOW_H

#include <QMainWindow>
#include "deanonwindow.h"

namespace Ui {
class AnonWindow;
}

class AnonWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit AnonWindow(QWidget *parent = nullptr);
    ~AnonWindow();

private slots:
    void on_back_button_clicked();
    void on_pushButton_clicked();
    //Anonymising Functions
    void KAnonymity();
    //void Aggregation();
    //void Differential();
    //void Randomwalkbased();
    //void Noiseaddition();
    //void Substitution();



    void on_anon_types_itemSelectionChanged();

    void on_anonButt_clicked();

    void on_next_button_clicked();

private:
    Ui::AnonWindow *ui;
    deanonwindow *deanonWin;
    //MainWindow *mainWin;
};

#endif // ANONWINDOW_H
