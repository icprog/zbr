#include "plotterdialog.h"
#include "ui_plotterdialog.h"
#include "mainwindow.h"
PlotterDialog::PlotterDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PlotterDialog)
{
    ui->setupUi(this);
    this->setFixedSize(size());
    this->setWindowTitle("Wykres współrzędnych maszynowych");
}

PlotterDialog::~PlotterDialog()
{
    delete ui;
}

void PlotterDialog::setPlotData(QList<machineCoordinates> *coords)
{
    fi1 = new QVector<double>();
    fi2 = new QVector<double>();
    fi3 = new QVector<double>();
    fi4 = new QVector<double>();
    fi5 = new QVector<double>();
    time = new QVector<double>();

    for(int i = 0; i<coords->size();i++)
    {
        fi1->append(MainWindow::radToDeg((coords->at(i)).fi1));
        fi2->append(MainWindow::radToDeg((coords->at(i)).fi2));
        fi3->append(MainWindow::radToDeg((coords->at(i)).fi3));
        fi4->append(MainWindow::radToDeg((coords->at(i)).fi4));
        fi5->append(MainWindow::radToDeg((coords->at(i)).fi5));
        time->append(i);
    }

    ui->plot->xAxis->setRange(0,time->last());
    ui->plot->yAxis->setRange(-1* 90, 90);
    ui->plot->xAxis->setTickLabels(false);
    ui->plot->addGraph();
    ui->plot->graph(0)->setData(*time,*fi1);
    ui->plot->graph(0)->setPen(QPen(Qt::red));
    ui->plot->graph(0)->setName("fi 1");
    ui->plot->addGraph();
    ui->plot->graph(1)->setData(*time,*fi2);
    ui->plot->graph(1)->setPen(QPen(Qt::blue));
    ui->plot->graph(1)->setName("fi 2");
    ui->plot->addGraph();
    ui->plot->graph(2)->setData(*time,*fi3);
    ui->plot->graph(2)->setPen(QPen(Qt::darkYellow));
    ui->plot->graph(2)->setName("fi 3");
    ui->plot->addGraph();
    ui->plot->graph(3)->setData(*time,*fi4);
    ui->plot->graph(3)->setPen(QPen(Qt::magenta));
    ui->plot->graph(3)->setName("fi 4");
    ui->plot->addGraph();
    ui->plot->graph(4)->setData(*time,*fi5);
    ui->plot->graph(4)->setPen(QPen(Qt::black));
    ui->plot->graph(4)->setName("fi 5");
    ui->plot->legend->setVisible(true);
    ui->plot->replot();

    delete fi1,fi2,fi3,fi4,fi5,time;
}
