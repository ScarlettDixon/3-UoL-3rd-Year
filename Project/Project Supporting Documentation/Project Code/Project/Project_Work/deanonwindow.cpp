#include "deanonwindow.h"
#include "ui_deanonwindow.h"
#include "csvlib.h"
#include <algorithm>
#include <QtAlgorithms>

deanonwindow::deanonwindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::deanonwindow)
{
    ui->setupUi(this);
    //int a,b,c;
    ui->deanonList->addItem("Singling Out");
    ui->deanonList->addItem("Linkability");
    ui->deanonList->addItem("Inference");
    ui->changeLab->setVisible(false);
    ui->changeText->setVisible(false);
    ui->backButt->setVisible(false);
}

deanonwindow::~deanonwindow()
{
    delete ui;
}



void deanonwindow::on_deanonList_itemSelectionChanged()
{
    int row = ui->deanonList->currentRow();
    switch (row){
    case 0:
        ui->descriptText->setText("When data is found to be unique and so can be used to identify the user");
        break;
    case 1:
        ui->descriptText->setText("Linking two or more records together, this can come from more than one source");
        break;
    case 2:
        ui->descriptText->setText("Using probability to relate two two or more values together");
        break;
    default:
        ui->descriptText->setText("");
        break;
    }
}

void deanonwindow::on_deanonButt_clicked()
{

    int row = ui->deanonList->currentRow();
    switch (row){
    case 0:
        singled();
        break;
    default:
        ui->descriptText->setText("Not available at this point in time, sorry for an inconvenience");
        ui->changeLab->setVisible(false);
        ui->changeLab->setText("Label:");
        ui->changeText->setVisible(false);
        ui->changeText->clear();
        break;
    }




    //deanit = std::unique(deanon.begin(), deanon.end());

}

void deanonwindow::singled(){
    QTextStream out(stdout);
    QVector<QVector<QString>> deanon = Store;

    //qsort(deanon);
    QVector<QString>::iterator deanit1;
    QVector<QString> temp;
    QString uniqData = "";
    int actualcol = 0;
    bool uniq = false;

    ui->changeLab->setVisible(true);
    ui->changeText->setVisible(true);
    ui->changeLab->setText("Column Data:");
    for (int col = 0; col < deanon[0].size(); col++ ){ //change col to 1 to avoid
        temp.clear();
        for (int row = 0; row < deanon.size(); row++){
        //out << col << "" << row << endl;
        temp.push_back(deanon[row][col]);
       }
       std::sort(temp.begin(), temp.end()); //sorting data to then find unique values
       deanit1 = std::unique(temp.begin(), temp.end());
       temp.resize( std::distance(temp.begin(),deanit1) );
       //temp.resize(deanit1->size());
       bool anonim = temp.contains("*");
       actualcol = col + 1;
       if (temp.size() == deanon.size() && anonim == false)
       {
           //out << "There are unique variables in column: " << col + 1 << endl;

           uniqData = uniqData + "There are unique variables in column: " + QString::number(actualcol) + "\n";
           uniq = true;

       }
       else if (anonim == true){
           uniqData = uniqData + "There are anonymized variables in column: " + QString::number(actualcol) + "\n";
       }
       else
       {
           //out << "There are non-unique variables in column: " << col + 1 << endl;
           uniqData = uniqData + "There are non-unique variables in column: " + QString::number(actualcol) + "\n";
       }


       //for (QVector<QString>::iterator it = temp.begin(); it!=temp.end(); ++it)
           //out << ' ' << *it;
    }
    if (uniq == true){uniqData = uniqData + "Warning! Unique data in data set, users can be identified!";}
    ui->changeText->setText(uniqData);

    //for(deanit1 = Store.begin(); deanit1 != Store.end(); deanit1++, col++)
    //{
        //temp = deanon[col];
       // out << temp[0];
        //out << deanon[0][col];
    //}

    //deanit =
    //qsor

    QStandardItemModel *model;
    model = new QStandardItemModel();

    model->setParent(this);
    ui->OutTab->setModel(model);
    ui->OutTab->setEditTriggers(QAbstractItemView::NoEditTriggers);

    Store = deanon;
    QVector<QVector<QString>>::iterator t1;
    QVector<QString>::iterator t2;
    //out << deanon[0][];
    int i= 0; int j = 0;
    for(t1 = Store.begin(); t1 != Store.end(); t1++, i++)    {
       for(t2 = t1->begin(); t2 != t1->end() ; t2++, j++) {
           int row = Store.size();
           int col = Store[0].size();
           model->setRowCount(row);
           model->setColumnCount(col);
           QModelIndex index = model->index(i,j,QModelIndex());
           model->setData(index, *t2);
       }
       j = 0;
    }

}

void deanonwindow::on_backButt_clicked()
{
    hide();
    //anonWin = new AnonWindow(this);
    //anonWin->show();


}
