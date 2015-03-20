#ifndef PLOTTERDIALOG_H
#define PLOTTERDIALOG_H

#include <QDialog>
#include "qcustomplot.h"
#include "typedefs.h"

namespace Ui {
class PlotterDialog;
}

class PlotterDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PlotterDialog(QWidget *parent = 0);
    ~PlotterDialog();
     void setPlotData(QList<machineCoordinates>* coords);

private:
    Ui::PlotterDialog *ui;
    QCustomPlot* plot;

//    QList<machineCoordinates>* coordinates;
    QVector<double>* fi1;
    QVector<double>* fi2;
    QVector<double>* fi3;
    QVector<double>* fi4;
    QVector<double>* fi5;
    QVector<double>* time;
};

#endif // PLOTTERDIALOG_H
