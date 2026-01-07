#include <volrover3/ThreadMonitorWidget.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QMessageBox>
#include <QDateTime>
#include <QMetaObject>

ThreadMonitorWidget::ThreadMonitorWidget(QWidget *parent)
    : QWidget(parent)
    , m_threadTable(nullptr)
    , m_updateTimer(nullptr)
    , m_cleanupTimer(nullptr)
    , m_updatePending(false)
{
    setupUI();
    
    // Set up rate-limiting timer (minimum 50ms between updates)
    m_updateTimer = new QTimer(this);
    m_updateTimer->setSingleShot(true);
    connect(m_updateTimer, &QTimer::timeout, this, &ThreadMonitorWidget::performUpdate);
    
    // Set up cleanup timer to remove completed threads after delay
    m_cleanupTimer = new QTimer(this);
    m_cleanupTimer->setInterval(10000);  // Check every 10 seconds
    connect(m_cleanupTimer, &QTimer::timeout, this, &ThreadMonitorWidget::cleanupCompletedThreads);
    m_cleanupTimer->start();
    
    // Start tracking time for rate limiting
    m_lastUpdateTime.start();
    
    // Register callbacks to be notified of thread changes
    registerCallbacks();
    
    // Initial population
    updateThreadTable();
}

ThreadMonitorWidget::~ThreadMonitorWidget()
{
    disconnectCallbacks();
    
    if (m_updateTimer) {
        m_updateTimer->stop();
    }
    if (m_cleanupTimer) {
        m_cleanupTimer->stop();
    }
}

void ThreadMonitorWidget::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    
    // Title label
    QLabel* titleLabel = new QLabel(tr("Active Threads"), this);
    QFont titleFont = titleLabel->font();
    titleFont.setPointSize(titleFont.pointSize() + 2);
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    mainLayout->addWidget(titleLabel);
    
    // Thread table
    m_threadTable = new QTableWidget(0, COL_COUNT, this);
    m_threadTable->setHorizontalHeaderLabels({
        tr("Thread Name"),
        tr("Status"),
        tr("Progress"),
        tr("Progress Bar"),
        tr("Action")
    });
    
    // Configure table
    m_threadTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_threadTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_threadTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_threadTable->verticalHeader()->setVisible(false);
    m_threadTable->setAlternatingRowColors(true);
    
    // Set column resize modes
    QHeaderView* header = m_threadTable->horizontalHeader();
    header->setSectionResizeMode(COL_NAME, QHeaderView::Stretch);
    header->setSectionResizeMode(COL_STATUS, QHeaderView::ResizeToContents);
    header->setSectionResizeMode(COL_PROGRESS, QHeaderView::ResizeToContents);
    header->setSectionResizeMode(COL_PROGRESS_BAR, QHeaderView::Fixed);
    header->setSectionResizeMode(COL_CANCEL, QHeaderView::ResizeToContents);
    header->resizeSection(COL_PROGRESS_BAR, 150);
    
    mainLayout->addWidget(m_threadTable);
    
    // Bottom info label
    QLabel* infoLabel = new QLabel(tr("Updates automatically on thread changes (rate limited to 50ms)"), this);
    infoLabel->setStyleSheet("color: gray; font-style: italic;");
    mainLayout->addWidget(infoLabel);
    
    setLayout(mainLayout);
    setMinimumSize(600, 300);
}

void ThreadMonitorWidget::registerCallbacks()
{
    // Connect to the app's thread map changes signal
    // This fires whenever a thread is added, removed, or its state changes
    // Use QMetaObject::invokeMethod to ensure UI updates happen on the main thread
    auto connection = cvc::app::instance().threadsChanged.connect(
        [this](const std::string&) {
            QMetaObject::invokeMethod(this, "requestUpdate", Qt::QueuedConnection);
        }
    );
    m_connections.push_back(connection);
}

void ThreadMonitorWidget::disconnectCallbacks()
{
    for (auto& conn : m_connections) {
        conn.disconnect();
    }
    m_connections.clear();
}

void ThreadMonitorWidget::requestUpdate()
{
    // Rate limiting: only update if at least 50ms has passed since last update
    const qint64 minUpdateInterval = 50; // milliseconds
    
    qint64 elapsed = m_lastUpdateTime.elapsed();
    
    if (elapsed >= minUpdateInterval) {
        // Enough time has passed, update immediately
        updateThreadTable();
        m_lastUpdateTime.restart();
        m_updatePending = false;
    } else {
        // Too soon, schedule an update for later if not already pending
        if (!m_updatePending) {
            m_updatePending = true;
            qint64 delay = minUpdateInterval - elapsed;
            m_updateTimer->start(static_cast<int>(delay));
        }
    }
}

void ThreadMonitorWidget::performUpdate()
{
    m_updatePending = false;
    updateThreadTable();
    m_lastUpdateTime.restart();
}

void ThreadMonitorWidget::updateThreadTable()
{
    // Get current threads from cvc::app
    cvc::thread_map threads = cvc::app::instance().threads();
    
    // Track which threads are completed (100% progress or thread finished)
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    for (const auto& entry : threads) {
        const std::string& threadKey = entry.first;
        const cvc::thread_ptr& thread = entry.second;
        
        if (!thread) continue;
        
        double progress = cvc::app::instance().threadProgress(threadKey);
        bool isComplete = (progress >= 1.0) || !thread->joinable();
        
        if (isComplete) {
            // Mark thread as completed if not already tracked
            if (m_completedThreads.find(threadKey) == m_completedThreads.end()) {
                m_completedThreads[threadKey] = currentTime;
                
                // Emit signal for status bar update
                std::string info = cvc::app::instance().threadInfo(threadKey);
                emit threadCompleted(QString::fromStdString(threadKey), 
                                   QString::fromStdString(info.empty() ? "completed" : info));
            }
        } else {
            // Thread is running again (restarted?), remove from completed tracking
            m_completedThreads.erase(threadKey);
        }
    }
    
    // Clear existing rows
    m_threadTable->setRowCount(0);
    
    // Add a row for each thread
    int row = 0;
    for (const auto& entry : threads) {
        const std::string& threadKey = entry.first;
        const cvc::thread_ptr& thread = entry.second;
        
        // Skip null threads
        if (!thread) continue;
        
        m_threadTable->insertRow(row);
        
        // Column 0: Thread name
        QTableWidgetItem* nameItem = new QTableWidgetItem(QString::fromStdString(threadKey));
        m_threadTable->setItem(row, COL_NAME, nameItem);
        
        // Column 1: Status (thread info)
        std::string statusInfo = cvc::app::instance().threadInfo(threadKey);
        double progress = cvc::app::instance().threadProgress(threadKey);
        bool isComplete = (progress >= 1.0) || !thread->joinable();
        
        if (isComplete) {
            statusInfo = statusInfo.empty() ? "completed" : statusInfo + " (completed)";
        } else if (statusInfo.empty()) {
            statusInfo = "running";
        }
        QTableWidgetItem* statusItem = new QTableWidgetItem(QString::fromStdString(statusInfo));
        
        // Color completed threads differently
        if (isComplete) {
            statusItem->setForeground(QColor(0, 128, 0));  // Green for completed
        }
        m_threadTable->setItem(row, COL_STATUS, statusItem);
        
        // Column 2: Progress percentage
        QString progressText = formatProgress(progress);
        QTableWidgetItem* progressItem = new QTableWidgetItem(progressText);
        progressItem->setTextAlignment(Qt::AlignCenter);
        if (isComplete) {
            progressItem->setForeground(QColor(0, 128, 0));
        }
        m_threadTable->setItem(row, COL_PROGRESS, progressItem);
        
        // Column 3: Progress bar
        QProgressBar* progressBar = new QProgressBar();
        progressBar->setRange(0, 100);
        progressBar->setValue(static_cast<int>(progress * 100));
        progressBar->setTextVisible(false);
        progressBar->setMaximumHeight(20);
        if (isComplete) {
            progressBar->setStyleSheet("QProgressBar::chunk { background-color: #4CAF50; }");
        }
        m_threadTable->setCellWidget(row, COL_PROGRESS_BAR, progressBar);
        
        // Column 4: Cancel button (disabled for completed threads)
        QPushButton* cancelBtn = new QPushButton(isComplete ? tr("Done") : tr("Cancel"));
        cancelBtn->setMaximumWidth(80);
        cancelBtn->setEnabled(!isComplete);
        
        // Capture threadKey by value for the lambda
        if (!isComplete) {
            connect(cancelBtn, &QPushButton::clicked, [this, threadKey]() {
                cancelThread(threadKey);
            });
        }
        
        m_threadTable->setCellWidget(row, COL_CANCEL, cancelBtn);
        
        row++;
    }
    
    // Adjust row heights
    for (int i = 0; i < m_threadTable->rowCount(); ++i) {
        m_threadTable->setRowHeight(i, 30);
    }
}

QString ThreadMonitorWidget::formatProgress(double progress)
{
    if (progress < 0.0) return tr("N/A");
    if (progress > 1.0) progress = 1.0;
    
    int percentage = static_cast<int>(progress * 100);
    return QString("%1%").arg(percentage);
}

void ThreadMonitorWidget::cancelThread(const std::string& threadKey)
{
    // Confirm cancellation
    int reply = QMessageBox::question(
        this,
        tr("Cancel Thread"),
        tr("Are you sure you want to cancel thread:\n%1?")
            .arg(QString::fromStdString(threadKey)),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No
    );
    
    if (reply != QMessageBox::Yes) {
        return;
    }
    
    // Get the thread and interrupt it
    cvc::thread_ptr thread = cvc::app::instance().threads(threadKey);
    if (thread) {
        thread->interrupt();
        
        QMessageBox::information(
            this,
            tr("Thread Cancelled"),
            tr("Cancellation request sent to thread:\n%1\n\n"
               "The thread will stop at its next interruption point.")
                .arg(QString::fromStdString(threadKey))
        );
    } else {
        QMessageBox::warning(
            this,
            tr("Thread Not Found"),
            tr("Thread %1 is no longer active.")
                .arg(QString::fromStdString(threadKey))
        );
    }
    
    // Request update (will be rate-limited)
    requestUpdate();
}

void ThreadMonitorWidget::cleanupCompletedThreads()
{
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    std::vector<std::string> threadsToRemove;
    
    // Find threads that have been completed for longer than the delay
    for (const auto& entry : m_completedThreads) {
        const std::string& threadKey = entry.first;
        qint64 completionTime = entry.second;
        
        if (currentTime - completionTime >= CLEANUP_DELAY_MS) {
            threadsToRemove.push_back(threadKey);
        }
    }
    
    // Remove the completed threads from the app's thread map
    for (const std::string& threadKey : threadsToRemove) {
        // Remove from our tracking
        m_completedThreads.erase(threadKey);
        
        // Remove from the app's thread map
        cvc::app::instance().removeThread(threadKey);
    }
    
    // Request UI update if we removed any threads
    if (!threadsToRemove.empty()) {
        requestUpdate();
    }
}
