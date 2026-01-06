#include <volrover3/StateTreeWidget.h>
#include <cvc/app.h>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QMessageBox>
#include <QRegularExpression>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <set>
#include <typeinfo>

StateTreeWidget::StateTreeWidget(QWidget *parent)
    : QWidget(parent)
    , m_rootState(nullptr)
    , m_currentState(nullptr)
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    
    // Create splitter for tree and table
    QSplitter* splitter = new QSplitter(Qt::Vertical, this);
    
    // Create tree widget for state hierarchy
    m_treeWidget = new QTreeWidget(this);
    m_treeWidget->setHeaderLabel(tr("State Tree"));
    m_treeWidget->setMinimumHeight(200);
    connect(m_treeWidget, &QTreeWidget::itemSelectionChanged,
            this, &StateTreeWidget::onTreeItemSelected);
    splitter->addWidget(m_treeWidget);
    
    // Create table widget for state properties
    m_tableWidget = new QTableWidget(this);
    m_tableWidget->setColumnCount(2);
    m_tableWidget->setHorizontalHeaderLabels({tr("Property"), tr("Value")});
    m_tableWidget->horizontalHeader()->setStretchLastSection(true);
    m_tableWidget->setMinimumHeight(150);
    connect(m_tableWidget, &QTableWidget::cellChanged,
            this, &StateTreeWidget::onTableValueChanged);
    splitter->addWidget(m_tableWidget);
    
    mainLayout->addWidget(splitter);
    
    // Create button bar
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    
    m_addButton = new QPushButton(tr("Add State..."), this);
    m_deleteButton = new QPushButton(tr("Delete State"), this);
    m_deleteButton->setEnabled(false);
    
    buttonLayout->addWidget(m_addButton);
    buttonLayout->addWidget(m_deleteButton);
    buttonLayout->addStretch();
    
    mainLayout->addLayout(buttonLayout);
    
    // Connect button signals
    connect(m_addButton, &QPushButton::clicked, this, &StateTreeWidget::onAddStateClicked);
    connect(m_deleteButton, &QPushButton::clicked, this, &StateTreeWidget::onDeleteStateClicked);
    
    // Set splitter sizes
    splitter->setSizes({300, 200});
}

StateTreeWidget::~StateTreeWidget()
{
    // Disconnect state change signal
    m_stateChangeConnection.disconnect();
}

void StateTreeWidget::setRootState(cvc::state* root)
{
    m_rootState = root;
    refresh();
}

void StateTreeWidget::refresh()
{
    m_treeWidget->clear();
    m_tableWidget->setRowCount(0);
    
    if (!m_rootState) return;
    
    // Create root item
    QTreeWidgetItem* rootItem = new QTreeWidgetItem(m_treeWidget);
    rootItem->setText(0, QString::fromStdString(m_rootState->name()));
    rootItem->setData(0, Qt::UserRole, QVariant::fromValue(static_cast<void*>(m_rootState)));
    
    // Populate tree recursively
    populateTree(rootItem, m_rootState, "");
    
    rootItem->setExpanded(true);
}

void StateTreeWidget::populateTree(QTreeWidgetItem* parentItem, cvc::state* state, const std::string& path)
{
    if (!state) return;
    
    try {
        // Get all descendant children (children() returns full paths and is recursive)
        std::vector<std::string> allChildren = state->children();
        
        // Get the parent's full name to filter for immediate children only
        std::string parentFullName = state->fullName();
        
        // Track immediate children (just the names, not full paths)
        std::set<std::string> immediateChildNames;
        
        for (const auto& childFullName : allChildren) {
            // Check if this is an immediate child by comparing paths
            // Immediate children will have parentFullName + "." + childName format
            // with no additional dots in childName
            
            if (childFullName.find(parentFullName) == 0) {
                // This child is under our parent node
                std::string relativePath = childFullName.substr(parentFullName.length());
                
                // Remove leading separator if present
                if (!relativePath.empty() && relativePath[0] == '.') {
                    relativePath = relativePath.substr(1);
                }
                
                // Check if this is an immediate child (no dots in relative path)
                if (!relativePath.empty() && relativePath.find('.') == std::string::npos) {
                    immediateChildNames.insert(relativePath);
                }
            }
        }
        
        // Now create tree items for each immediate child
        for (const auto& childName : immediateChildNames) {
            try {
                cvc::state& child = (*state)(childName);
                
                // Skip uninitialized states
                if (!child.initialized()) {
                    continue;
                }
                
                QTreeWidgetItem* childItem = new QTreeWidgetItem(parentItem);
                childItem->setText(0, QString::fromStdString(childName));
                childItem->setData(0, Qt::UserRole, QVariant::fromValue(static_cast<void*>(&child)));
                
                // Recursively populate this child's children
                populateTree(childItem, &child, childName);
            }
            catch (const std::exception& e) {
                // Child not accessible, skip
            }
        }
    }
    catch (const std::exception& e) {
        // No children or error getting children
    }
}

void StateTreeWidget::onTreeItemSelected()
{
    // Disconnect from previous state's change signal
    m_stateChangeConnection.disconnect();
    
    QList<QTreeWidgetItem*> selected = m_treeWidget->selectedItems();
    if (selected.isEmpty()) {
        m_tableWidget->setRowCount(0);
        m_currentState = nullptr;
        m_deleteButton->setEnabled(false);
        return;
    }
    
    QTreeWidgetItem* item = selected.first();
    void* statePtr = item->data(0, Qt::UserRole).value<void*>();
    m_currentState = static_cast<cvc::state*>(statePtr);
    
    // Enable delete button for non-root items
    m_deleteButton->setEnabled(m_currentState != m_rootState);
    
    populateTable(m_currentState);
    
    // Connect to new state's valueChanged signal to update UI when value changes
    if (m_currentState) {
        m_stateChangeConnection = m_currentState->valueChanged.connect([this]() {
            // Use Qt's queued connection to update UI from signal thread
            QMetaObject::invokeMethod(this, "onCurrentStateChanged", Qt::QueuedConnection);
        });
    }
}

void StateTreeWidget::onCurrentStateChanged()
{
    // Re-populate table with updated values from current state
    if (m_currentState) {
        populateTable(m_currentState);
    }
}

void StateTreeWidget::populateTable(cvc::state* state)
{
    if (!state) {
        m_tableWidget->setRowCount(0);
        return;
    }
    
    // Block signals while populating to avoid triggering cellChanged
    m_tableWidget->blockSignals(true);
    
    // Clear existing rows
    m_tableWidget->setRowCount(0);
    
    int row = 0;
    
    // Add "Name" row
    m_tableWidget->insertRow(row);
    QTableWidgetItem* nameLabel = new QTableWidgetItem(tr("name"));
    nameLabel->setFlags(nameLabel->flags() & ~Qt::ItemIsEditable);
    m_tableWidget->setItem(row, 0, nameLabel);
    
    QTableWidgetItem* nameItem = new QTableWidgetItem(QString::fromStdString(state->name()));
    nameItem->setFlags(nameItem->flags() & ~Qt::ItemIsEditable);
    nameItem->setForeground(QBrush(QColor(128, 128, 128)));
    m_tableWidget->setItem(row, 1, nameItem);
    row++;
    
    // Add "Full Path" row
    m_tableWidget->insertRow(row);
    QTableWidgetItem* pathLabel = new QTableWidgetItem(tr("full path"));
    pathLabel->setFlags(pathLabel->flags() & ~Qt::ItemIsEditable);
    m_tableWidget->setItem(row, 0, pathLabel);
    
    QTableWidgetItem* pathItem = new QTableWidgetItem(QString::fromStdString(state->fullName()));
    pathItem->setFlags(pathItem->flags() & ~Qt::ItemIsEditable);
    pathItem->setForeground(QBrush(QColor(128, 128, 128)));
    m_tableWidget->setItem(row, 1, pathItem);
    row++;
    
    // Add "Value" row
    m_tableWidget->insertRow(row);
    QTableWidgetItem* valueLabel = new QTableWidgetItem(tr("value"));
    valueLabel->setFlags(valueLabel->flags() & ~Qt::ItemIsEditable);
    m_tableWidget->setItem(row, 0, valueLabel);
    
    std::string valueStr = getStateValue(state);
    QTableWidgetItem* valueItem = new QTableWidgetItem(QString::fromStdString(valueStr));
    
    // Check if state is read-only and mark accordingly
    if (state->readOnly()) {
        // Make non-editable and add visual indicator
        valueItem->setFlags(valueItem->flags() & ~Qt::ItemIsEditable);
        valueItem->setForeground(QBrush(QColor(100, 100, 100)));
        valueItem->setToolTip(tr("This value is read-only (computed/generated)"));
        // Add lock emoji/icon to indicate read-only
        QString displayValue = QString::fromStdString(valueStr) + " 🔒";
        valueItem->setText(displayValue);
    }
    
    m_tableWidget->setItem(row, 1, valueItem);
    row++;
    
    // Add "Read-Only" status row
    m_tableWidget->insertRow(row);
    QTableWidgetItem* readOnlyLabel = new QTableWidgetItem(tr("read-only"));
    readOnlyLabel->setFlags(readOnlyLabel->flags() & ~Qt::ItemIsEditable);
    m_tableWidget->setItem(row, 0, readOnlyLabel);
    
    QString readOnlyStatus = state->readOnly() ? tr("Yes 🔒") : tr("No");
    QTableWidgetItem* readOnlyItem = new QTableWidgetItem(readOnlyStatus);
    readOnlyItem->setFlags(readOnlyItem->flags() & ~Qt::ItemIsEditable);
    if (state->readOnly()) {
        readOnlyItem->setForeground(QBrush(QColor(200, 100, 50)));
        readOnlyItem->setToolTip(tr("This state is read-only and cannot be modified"));
    } else {
        readOnlyItem->setForeground(QBrush(QColor(100, 150, 100)));
    }
    m_tableWidget->setItem(row, 1, readOnlyItem);
    row++;
    
    // Add "Comment" row if comment is set
    try {
        std::string commentStr = state->comment();
        if (!commentStr.empty()) {
            m_tableWidget->insertRow(row);
            QTableWidgetItem* commentLabel = new QTableWidgetItem(tr("comment"));
            commentLabel->setFlags(commentLabel->flags() & ~Qt::ItemIsEditable);
            m_tableWidget->setItem(row, 0, commentLabel);
            
            QTableWidgetItem* commentItem = new QTableWidgetItem(QString::fromStdString(commentStr));
            commentItem->setFlags(commentItem->flags() & ~Qt::ItemIsEditable);
            commentItem->setForeground(QBrush(QColor(80, 120, 180))); // Blue color for comments
            commentItem->setFont(QFont("", -1, QFont::Normal, true)); // Italic
            commentItem->setToolTip(QString::fromStdString(commentStr)); // Show full comment on hover
            m_tableWidget->setItem(row, 1, commentItem);
            row++;
        }
    } catch (...) {}
    
    // Add "Value Type" row if value type is set
    try {
        std::string valueTypeName = state->valueTypeName();
        if (!valueTypeName.empty()) {
            m_tableWidget->insertRow(row);
            QTableWidgetItem* valueTypeLabel = new QTableWidgetItem(tr("value type"));
            valueTypeLabel->setFlags(valueTypeLabel->flags() & ~Qt::ItemIsEditable);
            m_tableWidget->setItem(row, 0, valueTypeLabel);
            
            QTableWidgetItem* valueTypeItem = new QTableWidgetItem(QString::fromStdString(valueTypeName));
            valueTypeItem->setFlags(valueTypeItem->flags() & ~Qt::ItemIsEditable);
            valueTypeItem->setForeground(QBrush(QColor(128, 128, 128)));
            m_tableWidget->setItem(row, 1, valueTypeItem);
            row++;
        }
    } catch (...) {}
    
    // Add "Data Type" row only if data exists
    try {
        boost::any anyData = state->data();
        if (!anyData.empty()) {
            m_tableWidget->insertRow(row);
            QTableWidgetItem* dataLabel = new QTableWidgetItem(tr("data (type)"));
            dataLabel->setFlags(dataLabel->flags() & ~Qt::ItemIsEditable);
            m_tableWidget->setItem(row, 0, dataLabel);
            
            std::string dataType = getStateDataType(state);
            QTableWidgetItem* dataItem = new QTableWidgetItem(QString::fromStdString(dataType));
            dataItem->setFlags(dataItem->flags() & ~Qt::ItemIsEditable);
            dataItem->setForeground(QBrush(QColor(128, 128, 128)));
            m_tableWidget->setItem(row, 1, dataItem);
            row++;
        }
    } catch (...) {}
    
    // Add "Last Modified" row
    try {
        boost::posix_time::ptime lastMod = state->lastMod();
        if (!lastMod.is_not_a_date_time()) {
            m_tableWidget->insertRow(row);
            QTableWidgetItem* lastModLabel = new QTableWidgetItem(tr("last modified"));
            lastModLabel->setFlags(lastModLabel->flags() & ~Qt::ItemIsEditable);
            m_tableWidget->setItem(row, 0, lastModLabel);
            
            std::string timeStr = boost::posix_time::to_simple_string(lastMod);
            QTableWidgetItem* lastModItem = new QTableWidgetItem(QString::fromStdString(timeStr));
            lastModItem->setFlags(lastModItem->flags() & ~Qt::ItemIsEditable);
            lastModItem->setForeground(QBrush(QColor(128, 128, 128)));
            m_tableWidget->setItem(row, 1, lastModItem);
            row++;
        }
    } catch (...) {}
    
    // Add "Children Count" row
    m_tableWidget->insertRow(row);
    QTableWidgetItem* childrenLabel = new QTableWidgetItem(tr("children"));
    childrenLabel->setFlags(childrenLabel->flags() & ~Qt::ItemIsEditable);
    m_tableWidget->setItem(row, 0, childrenLabel);
    
    try {
        std::vector<std::string> children = state->children();
        QTableWidgetItem* childrenItem = new QTableWidgetItem(QString::number(children.size()));
        childrenItem->setFlags(childrenItem->flags() & ~Qt::ItemIsEditable);
        childrenItem->setForeground(QBrush(QColor(128, 128, 128)));
        m_tableWidget->setItem(row, 1, childrenItem);
    }
    catch (...) {
        QTableWidgetItem* childrenItem = new QTableWidgetItem(tr("0"));
        childrenItem->setFlags(childrenItem->flags() & ~Qt::ItemIsEditable);
        m_tableWidget->setItem(row, 1, childrenItem);
    }
    
    m_tableWidget->resizeColumnsToContents();
    m_tableWidget->blockSignals(false);
}

std::string StateTreeWidget::getStateValue(cvc::state* state)
{
    if (!state) return "";
    
    try {
        // state::value() returns std::string directly
        return state->value();
    }
    catch (const std::exception& e) {
        return std::string("<error: ") + e.what() + ">";
    }
}

std::string StateTreeWidget::getStateDataType(cvc::state* state)
{
    if (!state) return "unknown";
    
    try {
        // Get the boost::any data and use cvcapp to get the registered type name
        boost::any anyData = state->data();
        if (anyData.empty()) {
            return "<no data>";
        }
        
        // Use cvcapp's registered type names
        std::string typeName = cvcapp.dataTypeName(anyData);
        return typeName;
    }
    catch (const std::exception& e) {
        return "<no data>";
    }
}

void StateTreeWidget::setStateValue(cvc::state* state, const QString& valueStr)
{
    if (!state) return;
    
    try {
        std::string typeName = state->valueTypeName();
        
        // Try to set value based on detected type
        try {
            if (typeName.find("string") != std::string::npos) {
                state->value(valueStr.toStdString());
                return;
            }
        } catch (...) {}
        
        try {
            if (typeName.find("double") != std::string::npos || typeName == "d") {
                state->value(valueStr.toDouble());
                return;
            }
        } catch (...) {}
        
        try {
            if (typeName.find("float") != std::string::npos || typeName == "f") {
                state->value(valueStr.toFloat());
                return;
            }
        } catch (...) {}
        
        try {
            if (typeName.find("int") != std::string::npos || typeName == "i") {
                state->value(valueStr.toInt());
                return;
            }
        } catch (...) {}
        
        try {
            if (typeName == "j") { // unsigned int
                state->value(static_cast<unsigned int>(valueStr.toUInt()));
                return;
            }
        } catch (...) {}
        
        try {
            if (typeName.find("bool") != std::string::npos || typeName == "b") {
                QString lower = valueStr.toLower();
                bool boolValue = (lower == "true" || lower == "1" || lower == "yes");
                state->value(boolValue);
                return;
            }
        } catch (...) {}
        
        try {
            if (typeName == "l") { // long
                state->value(valueStr.toLong());
                return;
            }
        } catch (...) {}
        
        try {
            if (typeName == "m") { // size_t
                state->value(static_cast<size_t>(valueStr.toULongLong()));
                return;
            }
        } catch (...) {}
        
        // Default: try string
        state->value(valueStr.toStdString());
    }
    catch (const std::exception& e) {
        QMessageBox::warning(this, tr("Error Setting Value"),
                           tr("Failed to set value: %1").arg(e.what()));
    }
}

void StateTreeWidget::onTableValueChanged(int row, int column)
{
    if (!m_currentState || column != 1) return;
    
    // Only the value row (row 2) is editable (after name, full path)
    QTableWidgetItem* labelItem = m_tableWidget->item(row, 0);
    if (!labelItem || labelItem->text() != tr("value")) return;
    
    // Check if state is read-only before attempting to change
    if (m_currentState->readOnly()) {
        m_tableWidget->blockSignals(true);
        QMessageBox::information(this, tr("Read-Only State"),
                                tr("This state is read-only and cannot be modified.\n"
                                   "It contains computed or generated values."));
        // Revert to original value
        populateTable(m_currentState);
        m_tableWidget->blockSignals(false);
        return;
    }
    
    QTableWidgetItem* item = m_tableWidget->item(row, column);
    if (!item) return;
    
    QString newValue = item->text();
    
    // Block signals to prevent recursion
    m_tableWidget->blockSignals(true);
    
    try {
        setStateValue(m_currentState, newValue);
        // Value was set successfully, emit state changed signal
        emit stateChanged();
    }
    catch (const std::exception& e) {
        QMessageBox::warning(this, tr("Error"),
                           tr("Failed to update state value: %1").arg(e.what()));
        // Revert to old value
        populateTable(m_currentState);
    }
    
    m_tableWidget->blockSignals(false);
}

void StateTreeWidget::onAddStateClicked()
{
    // Pre-fill the path with the selected node's full name (if not root)
    QString prefill = "";
    if (m_currentState && m_currentState != m_rootState) {
        prefill = QString::fromStdString(m_currentState->fullName()) + ".";
    }
    
    bool ok;
    QString path = QInputDialog::getText(this, tr("Add State"),
                                        tr("Enter full path for new state:\ne.g., 'volrover3.my_setting' or 'myapp.nested.child.value'"),
                                        QLineEdit::Normal, prefill, &ok);
    
    if (!ok || path.isEmpty()) return;
    
    // Validate the path using state's validation function
    // Split path into components and validate each
    QStringList components = path.split('.');
    bool needsSanitization = false;
    QStringList sanitizedComponents;
    
    for (const QString& component : components) {
        std::string comp = component.toStdString();
        if (!cvc::state::isValidStateName(comp)) {
            needsSanitization = true;
            std::string sanitized = cvc::state::sanitizeStateName(comp);
            sanitizedComponents.append(QString::fromStdString(sanitized));
        } else {
            sanitizedComponents.append(component);
        }
    }
    
    // If path needs sanitization, offer to use sanitized version
    if (needsSanitization) {
        QString sanitizedPath = sanitizedComponents.join('.');
        QMessageBox::StandardButton reply = QMessageBox::question(this, tr("Invalid State Name"),
            tr("The path contains invalid characters.\n\n"
               "Original: %1\n"
               "Suggested: %2\n\n"
               "State names must follow C identifier rules:\n"
               "- Start with letter or underscore\n"
               "- Contain only letters, digits, and underscores\n"
               "- No spaces, dashes, or special characters\n\n"
               "Use sanitized version?")
            .arg(path).arg(sanitizedPath),
            QMessageBox::Yes | QMessageBox::No);
        
        if (reply == QMessageBox::Yes) {
            path = sanitizedPath;
        } else {
            return;
        }
    }
    
    QString value = QInputDialog::getText(this, tr("Add State"),
                                         tr("Enter initial value:"),
                                         QLineEdit::Normal, "", &ok);
    
    if (!ok) return;
    
    try {
        // Access the state using the full path from the global state singleton
        cvc::state& newState = cvc::state::instance()(path.toStdString());
        newState.value(value.toStdString());
        
        // Refresh the tree to show the new state
        refresh();
        
        QString fullPath = QString::fromStdString(newState.fullName());
        QMessageBox::information(this, tr("Success"),
                               tr("State '%1' created successfully").arg(fullPath));
    }
    catch (const std::exception& e) {
        QMessageBox::warning(this, tr("Error"),
                           tr("Failed to create state: %1").arg(e.what()));
    }
}

void StateTreeWidget::onDeleteStateClicked()
{
    if (!m_currentState || m_currentState == m_rootState) {
        QMessageBox::warning(this, tr("Error"),
                           tr("Cannot delete root state"));
        return;
    }
    
    QString stateName = QString::fromStdString(m_currentState->name());
    QString fullPath = QString::fromStdString(m_currentState->fullName());
    
    auto reply = QMessageBox::question(this, tr("Delete State"),
                                      tr("Are you sure you want to reset state '%1'?\n\nThis will clear its value, data, and mark it as uninitialized.").arg(fullPath),
                                      QMessageBox::Yes | QMessageBox::No);
    
    if (reply == QMessageBox::Yes) {
        try {
            // Reset the state (clears value, data, and sets initialized to false)
            m_currentState->reset();
            
            // Refresh the entire tree since the node should now be hidden
            refresh();
            
            // Emit state changed signal
            emit stateChanged();
            
            QMessageBox::information(this, tr("Success"),
                                   tr("State '%1' cleared successfully").arg(fullPath));
        }
        catch (const std::exception& e) {
            QMessageBox::warning(this, tr("Error"),
                               tr("Failed to clear state: %1").arg(e.what()));
        }
    }
}
