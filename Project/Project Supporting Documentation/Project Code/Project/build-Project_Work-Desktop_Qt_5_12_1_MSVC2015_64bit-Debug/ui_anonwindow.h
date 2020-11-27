/********************************************************************************
** Form generated from reading UI file 'anonwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ANONWINDOW_H
#define UI_ANONWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
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

class Ui_AnonWindow
{
public:
    QWidget *centralwidget;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QLabel *anonLab;
    QListWidget *anon_types;
    QPushButton *pushButton;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_3;
    QLabel *descriptLab;
    QTextBrowser *descriptText;
    QWidget *layoutWidget2;
    QVBoxLayout *verticalLayout_2;
    QLabel *anonLab_2;
    QListWidget *anon_types_2;
    QPushButton *anonButt;
    QWidget *layoutWidget3;
    QHBoxLayout *horizontalLayout;
    QPushButton *back_button;
    QPushButton *next_button;
    QWidget *layoutWidget4;
    QVBoxLayout *verticalLayout_4;
    QLabel *anonLab_3;
    QTableView *anon_Tab_Out;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *AnonWindow)
    {
        if (AnonWindow->objectName().isEmpty())
            AnonWindow->setObjectName(QString::fromUtf8("AnonWindow"));
        AnonWindow->resize(1162, 653);
        AnonWindow->setMinimumSize(QSize(984, 510));
        centralwidget = new QWidget(AnonWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        layoutWidget = new QWidget(centralwidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 0, 281, 211));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        anonLab = new QLabel(layoutWidget);
        anonLab->setObjectName(QString::fromUtf8("anonLab"));

        verticalLayout->addWidget(anonLab);

        anon_types = new QListWidget(layoutWidget);
        anon_types->setObjectName(QString::fromUtf8("anon_types"));

        verticalLayout->addWidget(anon_types);

        pushButton = new QPushButton(layoutWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        verticalLayout->addWidget(pushButton);

        layoutWidget1 = new QWidget(centralwidget);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(540, 0, 611, 211));
        verticalLayout_3 = new QVBoxLayout(layoutWidget1);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        descriptLab = new QLabel(layoutWidget1);
        descriptLab->setObjectName(QString::fromUtf8("descriptLab"));

        verticalLayout_3->addWidget(descriptLab);

        descriptText = new QTextBrowser(layoutWidget1);
        descriptText->setObjectName(QString::fromUtf8("descriptText"));

        verticalLayout_3->addWidget(descriptText);

        layoutWidget2 = new QWidget(centralwidget);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(310, 0, 201, 211));
        verticalLayout_2 = new QVBoxLayout(layoutWidget2);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        anonLab_2 = new QLabel(layoutWidget2);
        anonLab_2->setObjectName(QString::fromUtf8("anonLab_2"));

        verticalLayout_2->addWidget(anonLab_2);

        anon_types_2 = new QListWidget(layoutWidget2);
        anon_types_2->setObjectName(QString::fromUtf8("anon_types_2"));

        verticalLayout_2->addWidget(anon_types_2);

        anonButt = new QPushButton(layoutWidget2);
        anonButt->setObjectName(QString::fromUtf8("anonButt"));

        verticalLayout_2->addWidget(anonButt);

        layoutWidget3 = new QWidget(centralwidget);
        layoutWidget3->setObjectName(QString::fromUtf8("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(920, 560, 211, 41));
        horizontalLayout = new QHBoxLayout(layoutWidget3);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        back_button = new QPushButton(layoutWidget3);
        back_button->setObjectName(QString::fromUtf8("back_button"));

        horizontalLayout->addWidget(back_button);

        next_button = new QPushButton(layoutWidget3);
        next_button->setObjectName(QString::fromUtf8("next_button"));

        horizontalLayout->addWidget(next_button);

        layoutWidget4 = new QWidget(centralwidget);
        layoutWidget4->setObjectName(QString::fromUtf8("layoutWidget4"));
        layoutWidget4->setGeometry(QRect(10, 220, 1101, 331));
        verticalLayout_4 = new QVBoxLayout(layoutWidget4);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        anonLab_3 = new QLabel(layoutWidget4);
        anonLab_3->setObjectName(QString::fromUtf8("anonLab_3"));

        verticalLayout_4->addWidget(anonLab_3);

        anon_Tab_Out = new QTableView(layoutWidget4);
        anon_Tab_Out->setObjectName(QString::fromUtf8("anon_Tab_Out"));

        verticalLayout_4->addWidget(anon_Tab_Out);

        AnonWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(AnonWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1162, 21));
        AnonWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(AnonWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        AnonWindow->setStatusBar(statusbar);

        retranslateUi(AnonWindow);

        QMetaObject::connectSlotsByName(AnonWindow);
    } // setupUi

    void retranslateUi(QMainWindow *AnonWindow)
    {
        AnonWindow->setWindowTitle(QApplication::translate("AnonWindow", "MainWindow", nullptr));
        anonLab->setText(QApplication::translate("AnonWindow", "Anonymisation techniques:", nullptr));
        pushButton->setText(QApplication::translate("AnonWindow", "Choose Anonymisation type", nullptr));
        descriptLab->setText(QApplication::translate("AnonWindow", "Description of technique:", nullptr));
        anonLab_2->setText(QApplication::translate("AnonWindow", "Label:", nullptr));
        anonButt->setText(QApplication::translate("AnonWindow", "Choice Button", nullptr));
        back_button->setText(QApplication::translate("AnonWindow", "Back", nullptr));
        next_button->setText(QApplication::translate("AnonWindow", "Next", nullptr));
        anonLab_3->setText(QApplication::translate("AnonWindow", "Output:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class AnonWindow: public Ui_AnonWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ANONWINDOW_H
