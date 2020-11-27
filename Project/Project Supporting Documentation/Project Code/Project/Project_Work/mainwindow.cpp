#include "csvlib.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->DirecLab_2->setText(QDir::currentPath());
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_findFile_clicked()
{
    QTextStream out(stdout);
    Input inp;
    QString UserChoice = ui->file_usr_inp->text();
    QVector<QVector<QString>> output = inp.Reader(UserChoice);
    //out << output;
    Store = output;
    QStandardItemModel *model;
    model = new QStandardItemModel();

    model->setParent(this);
    ui->display_file->setModel(model);
    ui->display_file->setEditTriggers(QAbstractItemView::NoEditTriggers);

    QVector<QVector<QString>>::iterator t1;
    QVector<QString>::iterator t2;


    //bool ind = true;
    int i= 0; int j = 0;
    for(t1 = output.begin(); t1 != output.end(); t1++, i++ )    {
       for(t2 = t1->begin(); t2 != t1->end() ; t2++, j++) {
           int row = output.size();
           int col = output[0].size();
           model->setRowCount(row);
           model->setColumnCount(col);
           //int text = t1->
           QModelIndex index = model->index(i,j,QModelIndex());
           model->setData(index, *t2);
           //ui->textBrowser->setText(*t2);
           //ui->textBrowser->setText("Hello");
           //out<< col<< endl;
           //ui->menuBar->addMenu("Test");

        }
       j = 0;
    }
    //i = 0;
    //Anon An;
    //An.KAnonymity(output);
    //ui->display_file->update();
    //ui->display_file<< "test";
    //textstream m_TableHeader<<"#"<<"Name"<<"Text";
    //m_pTableWidget->setHorizontalHeaderLabels(m_TableHeader);


}

void MainWindow::on_next_button_clicked()
{
    hide();
    anonWin = new AnonWindow(this);
    anonWin->show();
}

