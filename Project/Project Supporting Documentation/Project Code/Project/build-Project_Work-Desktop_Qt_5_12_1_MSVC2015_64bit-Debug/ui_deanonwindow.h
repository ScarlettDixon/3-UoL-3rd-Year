/********************************************************************************
** Form generated from reading UI file 'deanonwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DEANONWINDOW_H
#define UI_DEANONWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTableView>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_deanonwindow
{
public:
    QWidget *centralwidget;
    QWidget *layoutWidget;
    QVBoxLayout *deanon_Lay;
    QLabel *deanonLab;
    QListWidget *deanonList;
    QPushButton *deanonButt;
    QWidget *layoutWidget_2;
    QVBoxLayout *deanon_DesLay;
    QLabel *descriptLab;
    QTextBrowser *descriptText;
    QWidget *layoutWidget_3;
    QVBoxLayout *deanon_OutLay;
    QLabel *OutLab;
    QTableView *OutTab;
    QWidget *layoutWidget_4;
    QVBoxLayout *deanon_DispLay;
    QLabel *changeLab;
    QTextBrowser *changeText;
    QPushButton *backButt;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *deanonwindow)
    {
        if (deanonwindow->objectName().isEmpty())
            deanonwindow->setObjectName(QString::fromUtf8("deanonwindow"));
        deanonwindow->resize(1176, 576);
        centralwidget = new QWidget(deanonwindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        layoutWidget = new QWidget(centralwidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(20, 20, 258, 242));
        deanon_Lay = new QVBoxLayout(layoutWidget);
        deanon_Lay->setObjectName(QString::fromUtf8("deanon_Lay"));
        deanon_Lay->setContentsMargins(0, 0, 0, 0);
        deanonLab = new QLabel(layoutWidget);
        deanonLab->setObjectName(QString::fromUtf8("deanonLab"));

        deanon_Lay->addWidget(deanonLab);

        deanonList = new QListWidget(layoutWidget);
        deanonList->setObjectName(QString::fromUtf8("deanonList"));

        deanon_Lay->addWidget(deanonList);

        deanonButt = new QPushButton(layoutWidget);
        deanonButt->setObjectName(QString::fromUtf8("deanonButt"));

        deanon_Lay->addWidget(deanonButt);

        layoutWidget_2 = new QWidget(centralwidget);
        layoutWidget_2->setObjectName(QString::fromUtf8("layoutWidget_2"));
        layoutWidget_2->setGeometry(QRect(290, 20, 221, 241));
        deanon_DesLay = new QVBoxLayout(layoutWidget_2);
        deanon_DesLay->setObjectName(QString::fromUtf8("deanon_DesLay"));
        deanon_DesLay->setContentsMargins(0, 0, 0, 0);
        descriptLab = new QLabel(layoutWidget_2);
        descriptLab->setObjectName(QString::fromUtf8("descriptLab"));

        deanon_DesLay->addWidget(descriptLab);

        descriptText = new QTextBrowser(layoutWidget_2);
        descriptText->setObjectName(QString::fromUtf8("descriptText"));

        deanon_DesLay->addWidget(descriptText);

        layoutWidget_3 = new QWidget(centralwidget);
        layoutWidget_3->setObjectName(QString::fromUtf8("layoutWidget_3"));
        layoutWidget_3->setGeometry(QRect(20, 270, 981, 221));
        deanon_OutLay = new QVBoxLayout(layoutWidget_3);
        deanon_OutLay->setObjectName(QString::fromUtf8("deanon_OutLay"));
        deanon_OutLay->setContentsMargins(0, 0, 0, 0);
        OutLab = new QLabel(layoutWidget_3);
        OutLab->setObjectName(QString::fromUtf8("OutLab"));

        deanon_OutLay->addWidget(OutLab);

        OutTab = new QTableView(layoutWidget_3);
        OutTab->setObjectName(QString::fromUtf8("OutTab"));

        deanon_OutLay->addWidget(OutTab);

        layoutWidget_4 = new QWidget(centralwidget);
        layoutWidget_4->setObjectName(QString::fromUtf8("layoutWidget_4"));
        layoutWidget_4->setGeometry(QRect(560, 20, 431, 231));
        deanon_DispLay = new QVBoxLayout(layoutWidget_4);
        deanon_DispLay->setObjectName(QString::fromUtf8("deanon_DispLay"));
        deanon_DispLay->setContentsMargins(0, 0, 0, 0);
        changeLab = new QLabel(layoutWidget_4);
        changeLab->setObjectName(QString::fromUtf8("changeLab"));

        deanon_DispLay->addWidget(changeLab);

        changeText = new QTextBrowser(layoutWidget_4);
        changeText->setObjectName(QString::fromUtf8("changeText"));

        deanon_DispLay->addWidget(changeText);

        backButt = new QPushButton(centralwidget);
        backButt->setObjectName(QString::fromUtf8("backButt"));
        backButt->setGeometry(QRect(940, 510, 75, 23));
        deanonwindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(deanonwindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1176, 21));
        deanonwindow->setMenuBar(menubar);
        statusbar = new QStatusBar(deanonwindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        deanonwindow->setStatusBar(statusbar);

        retranslateUi(deanonwindow);

        QMetaObject::connectSlotsByName(deanonwindow);
    } // setupUi

    void retranslateUi(QMainWindow *deanonwindow)
    {
        deanonwindow->setWindowTitle(QApplication::translate("deanonwindow", "MainWindow", nullptr));
        deanonLab->setText(QApplication::translate("deanonwindow", "Choose De-anonymizing technique:", nullptr));
        deanonButt->setText(QApplication::translate("deanonwindow", "Choose", nullptr));
        descriptLab->setText(QApplication::translate("deanonwindow", "Description of technique:", nullptr));
        OutLab->setText(QApplication::translate("deanonwindow", "Output:", nullptr));
        changeLab->setText(QApplication::translate("deanonwindow", "Label", nullptr));
        backButt->setText(QApplication::translate("deanonwindow", "Back", nullptr));
    } // retranslateUi

};

namespace Ui {
    class deanonwindow: public Ui_deanonwindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DEANONWINDOW_H
