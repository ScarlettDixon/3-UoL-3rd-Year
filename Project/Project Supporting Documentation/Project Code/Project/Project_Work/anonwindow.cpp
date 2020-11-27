#include "anonwindow.h"
#include "ui_anonwindow.h"
#include "csvlib.h"
#include "mainwindow.h"

AnonWindow::AnonWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::AnonWindow)
{
    ui->setupUi(this);

    ui->menubar->addMenu("File");

    //Adding in all anonymization technique options
    ui->anon_types->addItem("Aggregation - K-Anonymity");
    ui->anon_types->addItem("Class/Cluster");
    ui->anon_types->addItem("L-diversity");
    //ui->anon_types->addItem("Differential Privacy");
    //ui->anon_types->addItem("Random-walk-based methods");
    ui->anon_types->addItem("Noise addition");
    ui->anon_types->addItem("Substitution");
    ui->descriptText->setText("Choose a Technique for a description to appear here");

    ui->anonButt->setVisible(false);
    ui->anon_types_2->setVisible(false);
    ui->anonLab_2->setVisible(false);
    ui->back_button->setVisible(false);
    //ui->
}

AnonWindow::~AnonWindow()
{
    delete ui;
}




void AnonWindow::on_pushButton_clicked()
{
    //Anon an;

    int test = ui->anon_types->currentRow();
    switch (test){
    case 0:
        //an.KAnonymity(Anon_Lines);
        KAnonymity();
        break;
    default:
        ui->descriptText->setText("Not available at this point in time, sorry for an inconvenience");
        ui->anonButt->setVisible(false); //Implimented after test number 2.4
        ui->anonLab_2->setVisible(false);
        ui->anon_types_2->setVisible(false);
        break;
    }
//if (ui->anon_types->currentIndex() == 0)

}

void AnonWindow::on_anon_types_itemSelectionChanged()
{
    int row = ui->anon_types->currentRow();
    switch (row){
    case 0:
        ui->descriptText->setText("Obscuring Data considered quasi-identifier attributes (Name, Postcode, Gender etc)");
        break;
    case 1:
        ui->descriptText->setText("Grouping similar user data to reduce how identifiable the data is");
        break;
    case 2:
        ui->descriptText->setText("An extension of K-Anonymity that aims to resolve the weaknesses created by it");
        break;
    case 3:
        ui->descriptText->setText("Taking precise initial data and adding imprecision (e.g. height = 5’7 replaced with 5'4 + or - 5 inches)");
        break;
    case 4:
        ui->descriptText->setText("Replacing an identifiable value with a value not linked to the original (e.g. height = 5’7 replaced with the colour blue)");
        break;
    case 5:
        ui->descriptText->setText("Using statistical probability to manipulate data in such a way that the possibility of deanonymization is stopped while also being able to reverse the manipulation to retain utility");
        break;
    default:
        ui->descriptText->setText("Choose a Technique for a description to appear here");
        break;
    }

}

void AnonWindow::KAnonymity()
{
    ui->anonButt->setVisible(true);
    ui->anonLab_2->setVisible(true);
    ui->anon_types_2->setVisible(true);
    ui->anon_types_2->clear();
    ui->anonLab_2->setText("Header:");
    ui->anonButt->setText("Choose Identifiable Feature");
    //QVector<QVector<QString>> Anon_Lines = Store;
    //QTextStream out(stdout);
    foreach (QString H, Heads){
     ui->anon_types_2->addItem(H);
     //QObject::connect(ui->anonButt, );
    }
}

void AnonWindow::on_anonButt_clicked()
{
    int row1 = ui->anon_types->currentRow();
    int row2 = ui->anon_types_2->currentRow();
    int stsz = Store.size();
    QVector<QVector<QString>> Temp = Store;
    switch (row1){
    case 0:

       for (int i = 1; i < stsz; i++)
        {
           Store[i][row2] = "*";
        }
        break;
    case 1:
        break;
    default:
        break;
    }


    QStandardItemModel *model;
    model = new QStandardItemModel();

    model->setParent(this);
    ui->anon_Tab_Out->setModel(model);
    ui->anon_Tab_Out->setEditTriggers(QAbstractItemView::NoEditTriggers);

    QVector<QVector<QString>>::iterator t1;
    QVector<QString>::iterator t2;
    QTextStream out(stdout);
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
    //Store = Temp;

}

/*Anonymizing techniques:
K-Anonymity – Obscuring Data considered quasi-identifier attributes (Name, Postcode, Gender etc)
Aggregation/Class/Cluster – Grouping similar user data to reduce ability to identify (K-anon is a type of aggregate); L-diversity is interesting
L-Diversity -
Differential Privacy – used with third parties, third party only gets anonymised data set, techniques are applied to original company to stop de-anonymization
Random-walk-based methods – preserving link privacy, edge is replaced by a RW Path
Noise addition – Adding imprecision to the precise original data
Substitution – Replacing an important value with something else (e.g. height = 5’7 replaced with colour blue), used with noise addition a lot

Pseudononymization techniques:
Does not remove all identifiable info just reduces linkability
Hash Functions – hash data to a fixed size
Tokenization – similar to substitution but the data is traceable back to the original, unique substitutions*/


void AnonWindow::on_back_button_clicked()
{
    //deanonWin
    //mainWin = new MainWindow(this);
}

void AnonWindow::on_next_button_clicked()
{
    hide();
    deanonWin = new deanonwindow(this);
    deanonWin->show();
}
