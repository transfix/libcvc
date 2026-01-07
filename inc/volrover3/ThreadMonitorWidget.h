#ifndef VOLROVER3_THREADMONITORWIDGET_H
#define VOLROVER3_THREADMONITORWIDGET_H

#include <QWidget>
#include <QTableWidget>
#include <QTimer>
#include <QPushButton>
#include <QProgressBar>
#include <QElapsedTimer>
#include <boost/signals2/connection.hpp>
#include <cvc/app.h>
#include <map>

class ThreadMonitorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ThreadMonitorWidget(QWidget *parent = nullptr);
    ~ThreadMonitorWidget();

signals:
    // Emitted when a thread completes, with thread name and info for status bar
    void threadCompleted(const QString& threadName, const QString& threadInfo);

public slots:
    void requestUpdate();
    void performUpdate();
    void cancelThread(const std::string& threadKey);
    void cleanupCompletedThreads();

private:
    void setupUI();
    void updateThreadTable();
    void registerCallbacks();
    void disconnectCallbacks();
    QString formatProgress(double progress);
    
    QTableWidget* m_threadTable;
    QTimer* m_updateTimer;
    QTimer* m_cleanupTimer;  // Timer to remove completed threads after delay
    QElapsedTimer m_lastUpdateTime;
    bool m_updatePending;
    
    // Track when threads completed (thread key -> completion timestamp)
    std::map<std::string, qint64> m_completedThreads;
    static const int CLEANUP_DELAY_MS = 60000;  // Remove completed threads after 60 seconds
    
    // Callback connections for cleanup
    std::vector<boost::signals2::connection> m_connections;
    
    // Column indices
    enum Column {
        COL_NAME = 0,
        COL_STATUS = 1,
        COL_PROGRESS = 2,
        COL_PROGRESS_BAR = 3,
        COL_CANCEL = 4,
        COL_COUNT = 5
    };
};

#endif // VOLROVER3_THREADMONITORWIDGET_H
