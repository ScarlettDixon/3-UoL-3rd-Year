/*
 *
 */

#include <fstream>
#include <sstream>
#include "csvlib.h"

bool fileExists(QString path) {
    QFileInfo check_file(path);
    // check if file exists and if yes: Is it really a file and no directory?
    if (check_file.exists() && check_file.isFile()) {
        return true;
    } else {
        return false;
    }
}

void Filecheck(){
    #ifdef Q_OS_UNIX

    QFileInfo info1("/home/bob/bin/untabify");
    info1.isSymLink();          // returns true
    info1.absoluteFilePath();   // returns "/home/bob/bin/untabify"
    info1.size();               // returns 56201
    info1.symLinkTarget();      // returns "/opt/pretty++/bin/untabify"

    QFileInfo info2(info1.symLinkTarget());
    info2.isSymLink();          // returns false
    info2.absoluteFilePath();   // returns "/opt/pretty++/bin/untabify"
    info2.size();               // returns 56201

    #endif

    #ifdef Q_OS_WIN

    QFileInfo info1("C:\\Documents and Settings\\Bob\\untabify.lnk");
    info1.isSymLink();          // returns true
    info1.absoluteFilePath();   // returns "C:/Documents and Settings/Bob/untabify.lnk"
    info1.size();               // returns 743
    info1.symLinkTarget();      // returns "C:/Pretty++/untabify"

    QFileInfo info2(info1.symLinkTarget());
    info2.isSymLink();          // returns false
    info2.absoluteFilePath();   // returns "C:/Pretty++/untabify"
    info2.size();               // returns 63942

    #endif
}

//deque<string> * Input::Reader() {
QVector<QVector<QString>> Input::Reader(QString inp_File) {
	Input File1;
    QFile file(inp_File);
    if (file.exists() != true){
        file.flush();
        file.setFileName(File1.filenm);
    }
    file.open(QIODevice::ReadOnly);
    //QString filePath = QDir::currentPath() + "/test.csv";
    //cout << a.filenm << endl;
	//File1.filenm = "src/test.csv"; //Replace with GUI in the future
    //ifstream file (File1.filenm);
    //QString t1 = "Hello";
    //int count = 0;
   // if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
       // error message here
    //   return 34;
       //stringstream test1;
   // }
    //bool is_filethere = fileExists(File1.filenm);
    //Filecheck();
    //QString val, val2;
    QTextStream out(stdout);
    out << QDir::currentPath() << endl;
    /*//out << QDir::Size(file) << endl;
    while (!file.atEnd())
	{
        QVector<QString> Words;
        QString line = file.readLine();
        QStringList list = line.split(",");
        foreach(QString test, list){
        Words.push_back(test);
        out << test << endl;
        }
        File1.Lines.push_back(Words);
    }*/
       /* //vector <string> Words;
        getline(file, val, '\n');
        test1 << val;
        while(test1.good()){
            getline(test1, val2, ',');
            Words.push_back(val2);
        }*/
        //test1.str("");
        //test1.clear();
        //cout << val << endl;
    //out << File1.Lines.at(1).at(1)<< endl;
	//cout << File1.Lines[2][1] << endl;
	//cout << File1.Lines.size() << endl;
	//cout << File1.Lines[0];
    //deque<string> FilePoint = File1.Lines;
    int count = 0;
    QVector<QString> Head_Holder;
    QTextStream in(&file);
        while (!in.atEnd()) {
            QVector<QString> Words;
            QString line = in.readLine();
            QStringList list = line.split(",");
            foreach(QString test, list){
            if (count == 0){Head_Holder.push_back(test);}
            Words.push_back(test);
            //out << test << endl;
            //out << line << endl;
            }
            count+=1;
            File1.Lines.push_back(Words);
    }

    //Anon An1;
    file.close();
    Heads = Head_Holder;
    //An1.KAnonymity(File1.Lines);
    return File1.Lines;
}

