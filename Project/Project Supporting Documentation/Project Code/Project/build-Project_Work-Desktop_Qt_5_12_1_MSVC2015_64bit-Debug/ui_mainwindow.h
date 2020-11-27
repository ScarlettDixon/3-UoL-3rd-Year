/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTableView>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QLabel *InpFileLab;
    QLineEdit *file_usr_inp;
    QPushButton *findFile;
    QPushButton *next_button;
    QTableView *display_file;
    QLabel *DirecLab_1;
    QLabel *DirecLab_2;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(858, 448);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 0, 371, 41));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        InpFileLab = new QLabel(layoutWidget);
        InpFileLab->setObjectName(QString::fromUtf8("InpFileLab"));

        horizontalLayout->addWidget(InpFileLab);

        file_usr_inp = new QLineEdit(layoutWidget);
        file_usr_inp->setObjectName(QString::fromUtf8("file_usr_inp"));

        horizontalLayout->addWidget(file_usr_inp);

        findFile = new QPushButton(layoutWidget);
        findFile->setObjectName(QString::fromUtf8("findFile"));

        horizontalLayout->addWidget(findFile);

        next_button = new QPushButton(centralWidget);
        next_button->setObjectName(QString::fromUtf8("next_button"));
        next_button->setGeometry(QRect(360, 360, 88, 34));
        display_file = new QTableView(centralWidget);
        display_file->setObjectName(QString::fromUtf8("display_file"));
        display_file->setGeometry(QRect(20, 70, 811, 281));
        DirecLab_1 = new QLabel(centralWidget);
        DirecLab_1->setObjectName(QString::fromUtf8("DirecLab_1"));
        DirecLab_1->setGeometry(QRect(10, 50, 271, 18));
        DirecLab_2 = new QLabel(centralWidget);
        DirecLab_2->setObjectName(QString::fromUtf8("DirecLab_2"));
        DirecLab_2->setGeometry(QRect(260, 50, 461, 21));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 858, 21));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        InpFileLab->setText(QApplication::translate("MainWindow", "Input File:", nullptr));
        findFile->setText(QApplication::translate("MainWindow", "Browse", nullptr));
        next_button->setText(QApplication::translate("MainWindow", "Next", nullptr));
        DirecLab_1->setText(QApplication::translate("MainWindow", "The program will be looking at directory: ", nullptr));
        DirecLab_2->setText(QApplication::translate("MainWindow", "Dir", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
