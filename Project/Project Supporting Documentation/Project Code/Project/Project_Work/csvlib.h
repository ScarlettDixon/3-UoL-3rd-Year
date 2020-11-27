/*
 *
 */
#pragma once
#ifndef CSVLIB_H_
#define CSVLIB_H_
#include <iostream>
#include <deque>
#include <vector>
#include <QTextStream>
#include <QVector>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QTableView>
#include <QStandardItemModel>
#include <QMainWindow>
using namespace std;


extern QVector<QVector<QString>> Store;
extern QVector<QString> Heads;

class Input{
	public:
    QVector<QVector<QString>> Reader(QString inp_File);
	//deque<string> * Reader();
	//deque<deque> EntireFile;
    QVector <QVector <QString>> Lines;
	//deque<string> Lines;
	//test1 <string> class vector;
    Input(){ filenm = "test.csv";}
    ~Input(){}
	private:
    QString filenm;
};

class Anon: public Input{
	public:
    void KAnonymity(QVector <QVector <QString> > INP);
	void Aggregation();
	void Differential();
	void Randomwalkbased();
	void Noiseaddition();
	void Substitution();
	Anon(){}
	~Anon(){}
	private:
};

class DeAnon{
	public:
	DeAnon(){}
	~DeAnon(){}
	void Seedbased();
	void Community();
	void Seedfree();
	private:
};

class Output{
	public:
	Output(){}
	~Output(){}
	void GUIDisplay();
	private:
};

#endif /* CSVLIB_H_ */
