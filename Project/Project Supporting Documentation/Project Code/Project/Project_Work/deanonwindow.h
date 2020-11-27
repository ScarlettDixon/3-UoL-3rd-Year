#ifndef DEANONWINDOW_H
#define DEANONWINDOW_H

#include <QMainWindow>

namespace Ui {
class deanonwindow;
}

class deanonwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit deanonwindow(QWidget *parent = nullptr);
    ~deanonwindow();


private slots:

    void on_deanonList_itemSelectionChanged();
    void on_deanonButt_clicked();
    void singled();

    void on_backButt_clicked();

private:
    Ui::deanonwindow *ui;
};

#endif // DEANONWINDOW_H
