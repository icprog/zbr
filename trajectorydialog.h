#ifndef TRAJECTORYDIALOG_H
#define TRAJECTORYDIALOG_H


#include <QDialog>
#include "typedefs.h"
#include "trajectory.h"

namespace Ui {
class TrajectoryDialog;
}

class TrajectoryDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TrajectoryDialog(QWidget *parent = 0);
    ~TrajectoryDialog();
    void reset();

private:
    Ui::TrajectoryDialog *ui;
    Trajectory *trajectory;

signals:
    void trajectoryEditFinished(Trajectory *trajectory);
private slots:
    void on_addPointButton_clicked();
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();
};

#endif // TRAJECTORYDIALOG_H
