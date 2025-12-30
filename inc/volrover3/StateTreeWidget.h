#ifndef STATETREEWIDGET_H
#define STATETREEWIDGET_H

#include <QWidget>
#include <QTreeWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QSplitter>
#include <cvc/state.h>
#include <string>
#include <vector>

class StateTreeWidget : public QWidget
{
    Q_OBJECT

public:
    explicit StateTreeWidget(QWidget *parent = nullptr);
    ~StateTreeWidget() override = default;

    void setRootState(cvc::state* root);
    void refresh();

private slots:
    void onTreeItemSelected();
    void onTableValueChanged(int row, int column);
    void onAddStateClicked();
    void onDeleteStateClicked();

private:
    void populateTree(QTreeWidgetItem* parentItem, cvc::state* state, const std::string& path);
    void populateTable(cvc::state* state);
    std::string getStateValue(cvc::state* state);
    std::string getStateDataType(cvc::state* state);
    void setStateValue(cvc::state* state, const QString& valueStr);

    QTreeWidget* m_treeWidget;
    QTableWidget* m_tableWidget;
    QPushButton* m_addButton;
    QPushButton* m_deleteButton;
    cvc::state* m_rootState;
    cvc::state* m_currentState;
};

#endif // STATETREEWIDGET_H
