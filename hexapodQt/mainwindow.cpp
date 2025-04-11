#include <QMessageBox>
#include <QDateTime>
#include <QFile>
#include <QColorDialog>
#include <QInputDialog>
#include <QCloseEvent>
#include <QKeySequence>
#include <QShortcut>
#include <QStyle>  // Add this include for QStyle
#include <QAction> // Add this include for QAction
#include "hexapodprotocol.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), m_connection(new HexapodConnection(this)), m_imuUpdateTimer(new QTimer(this)), m_keyProcessTimer(new QTimer(this)), m_settings(QSettings::IniFormat, QSettings::UserScope, "HexapodRobotics", "HexapodController"), m_forwardPressed(false), m_backwardPressed(false), m_leftPressed(false), m_rightPressed(false), m_darkMode(false)
{
    ui->setupUi(this);

    // Set window properties
    setWindowTitle("Hexapod Remote Control");
    setWindowIcon(QIcon(":/icons/hexapod.png"));

    // Create theme action programmatically since it doesn't exist in UI
    QAction *actionTheme = new QAction("Dark Theme", this);
    actionTheme->setCheckable(true);
    actionTheme->setChecked(m_darkMode);
    connect(actionTheme, &QAction::toggled, this, &MainWindow::on_actionTheme_toggled);

    // Add to View menu if it exists, otherwise create a new menu
    QMenu *viewMenu = menuBar()->findChild<QMenu *>("menuView");
    if (!viewMenu)
    {
        viewMenu = menuBar()->addMenu("&View");
    }
    viewMenu->addAction(actionTheme);

    // Create keyboard shortcut menu action
    QAction *actionKeyboardShortcuts = new QAction("Keyboard Shortcuts", this);
    connect(actionKeyboardShortcuts, &QAction::triggered, this, &MainWindow::on_actionKeyboardShortcuts_triggered);
    viewMenu->addAction(actionKeyboardShortcuts);

    // Store these actions as member variables to access them later
    m_actionTheme = actionTheme;

    // Connect signals from HexapodConnection
    connect(m_connection, &HexapodConnection::connectionStatusChanged, this, &MainWindow::handleConnectionStatus);
    connect(m_connection, &HexapodConnection::responseReceived, this, &MainWindow::handleResponseReceived);
    connect(m_connection, &HexapodConnection::imuDataReceived, this, &MainWindow::handleImuDataReceived);
    connect(m_connection, &HexapodConnection::errorOccurred, this, &MainWindow::handleErrorOccurred);

    // Set up IMU update timer
    m_imuUpdateTimer->setInterval(100); // 10 Hz update rate
    connect(m_imuUpdateTimer, &QTimer::timeout, this, &MainWindow::requestImuUpdate);

    // Set up keyboard processing timer
    m_keyProcessTimer->setInterval(50); // 20 Hz update rate
    connect(m_keyProcessTimer, &QTimer::timeout, this, &MainWindow::updateMovement);

    // Initial UI setup
    ui->statusLabel->setText("Disconnected");
    ui->statusLabel->setProperty("status", "disconnected");
    ui->statusLabel->setStyleSheet(""); // Will be set by theme

    // Set up sliders
    ui->speedSlider->setRange(0, 100);
    ui->speedSlider->setValue(50);
    ui->speedSlider->setEnabled(false);

    ui->pitchSlider->setRange(-30, 30);
    ui->pitchSlider->setValue(0);
    ui->pitchSlider->setEnabled(false);

    ui->rollSlider->setRange(-30, 30);
    ui->rollSlider->setValue(0);
    ui->rollSlider->setEnabled(false);

    // Set up buttons
    ui->forwardButton->setEnabled(false);
    ui->backwardButton->setEnabled(false);
    ui->leftButton->setEnabled(false);
    ui->rightButton->setEnabled(false);
    ui->stopButton->setEnabled(false);

    // Set up balance checkbox
    ui->balanceCheckbox->setEnabled(false);

    // Setup keyboard shortcuts
    QShortcut *themeShortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_T), this);
    connect(themeShortcut, &QShortcut::activated, this, &MainWindow::toggleTheme);

    QShortcut *helpShortcut = new QShortcut(QKeySequence(Qt::Key_F1), this);
    connect(helpShortcut, &QShortcut::activated, this, &MainWindow::showKeyboardShortcutsDialog);

    // Load settings
    loadSettings();

    // Apply theme
    applyTheme();

    // Log initial message
    logMessage("Application started. Connect to your Hexapod to begin control.");

    // Setup keyboard focus and controls
    setFocusPolicy(Qt::StrongFocus);
    setupKeyboardControls();
}

MainWindow::~MainWindow()
{
    if (m_connection && m_connection->isConnected())
    {
        m_connection->disconnect();
    }

    // Save settings before closing
    saveSettings();

    delete ui;
}

void MainWindow::loadSettings()
{
    // Load connection settings
    QString host = m_settings.value("connection/host", "localhost").toString();
    int port = m_settings.value("connection/port", 8080).toInt();

    ui->hostEdit->setText(host);
    ui->portSpin->setValue(port);

    // Load theme preference
    m_darkMode = m_settings.value("appearance/darkMode", false).toBool();

    // Update action state
    if (m_actionTheme)
    {
        m_actionTheme->setChecked(m_darkMode);
    }

    // Load visualization settings
    bool showCoords = m_settings.value("visualization/showCoordinates", true).toBool();
    bool highlightLegs = m_settings.value("visualization/highlightLegs", true).toBool();
    ui->visualizationWidget->setShowCoordinateSystem(showCoords);
    ui->visualizationWidget->setHighlightActiveLeg(highlightLegs);
}

void MainWindow::saveSettings()
{
    // Save connection settings
    m_settings.setValue("connection/host", ui->hostEdit->text());
    m_settings.setValue("connection/port", ui->portSpin->value());

    // Save theme preference
    m_settings.setValue("appearance/darkMode", m_darkMode);

    // Save visualization settings
    // These would typically be tied to UI controls but simplified here
    m_settings.setValue("visualization/showCoordinates", true);
    m_settings.setValue("visualization/highlightLegs", true);

    m_settings.sync();
}

void MainWindow::applyTheme()
{
    // Apply theme to visualization widget
    ui->visualizationWidget->setColorScheme(m_darkMode);

    // Load appropriate stylesheet
    QFile styleFile;
    if (m_darkMode)
    {
        styleFile.setFileName(":/styles/hexapod_dark.qss");
    }
    else
    {
        styleFile.setFileName(":/styles/hexapod.qss");
    }

    if (styleFile.open(QFile::ReadOnly | QFile::Text))
    {
        QString style = QLatin1String(styleFile.readAll());
        setStyleSheet(style);
        styleFile.close();
    }
    else
    {
        logError("Failed to load theme stylesheet");
    }

    // Update status indicators by directly setting properties
    if (m_connection && m_connection->isConnected())
    {
        ui->statusLabel->setProperty("status", "connected");
    }
    else
    {
        ui->statusLabel->setProperty("status", "disconnected");
    }

    // Force a style update by asking the widget to update itself
    ui->statusLabel->update();
}

void MainWindow::toggleTheme()
{
    m_darkMode = !m_darkMode;
    if (m_actionTheme)
    {
        m_actionTheme->setChecked(m_darkMode);
    }
    applyTheme();
    logInfo(QString("Theme changed to %1").arg(m_darkMode ? "dark" : "light"));
}

void MainWindow::on_actionTheme_toggled(bool darkMode)
{
    if (m_darkMode != darkMode)
    {
        m_darkMode = darkMode;
        applyTheme();
    }
}

void MainWindow::showKeyboardShortcutsDialog()
{
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Keyboard Shortcuts");
    msgBox.setTextFormat(Qt::RichText);

    QString shortcutsHtml = "<h3>Keyboard Controls</h3>"
                            "<table>"
                            "<tr><th>Key</th><th>Action</th></tr>"
                            "<tr><td>W</td><td>Move Forward</td></tr>"
                            "<tr><td>S</td><td>Move Backward</td></tr>"
                            "<tr><td>A</td><td>Turn Left</td></tr>"
                            "<tr><td>D</td><td>Turn Right</td></tr>"
                            "<tr><td>Space</td><td>Stop All Movement</td></tr>"
                            "<tr><td>B</td><td>Toggle Balance Mode</td></tr>"
                            "<tr><td>F1</td><td>This Help Dialog</td></tr>"
                            "<tr><td>Ctrl+T</td><td>Toggle Dark/Light Theme</td></tr>"
                            "</table>";

    msgBox.setText(shortcutsHtml);
    msgBox.setIcon(QMessageBox::Information);
    msgBox.exec();
}

void MainWindow::on_actionKeyboardShortcuts_triggered()
{
    showKeyboardShortcutsDialog();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    // Ensure we save settings on close
    saveSettings();

    // If connected, ask if user wants to disconnect
    if (m_connection && m_connection->isConnected())
    {
        QMessageBox::StandardButton reply = QMessageBox::question(
            this, "Disconnect", "Disconnect from robot before exiting?",
            QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);

        if (reply == QMessageBox::Cancel)
        {
            event->ignore();
            return;
        }
        else if (reply == QMessageBox::Yes)
        {
            m_connection->disconnect();
        }
    }

    event->accept();
}

void MainWindow::setupKeyboardControls()
{
    // We'll implement key handling in keyPressEvent and keyReleaseEvent

    // Set up focus
    setFocus();
    grabKeyboard();
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    // Check if not auto-repeated key press
    if (!event->isAutoRepeat())
    {
        switch (event->key())
        {
        case Qt::Key_W:
            m_forwardPressed = true;
            updateMovement();
            break;
        case Qt::Key_S:
            m_backwardPressed = true;
            updateMovement();
            break;
        case Qt::Key_A:
            m_leftPressed = true;
            updateMovement();
            break;
        case Qt::Key_D:
            m_rightPressed = true;
            updateMovement();
            break;
        case Qt::Key_Space:
            on_stopButton_clicked();
            break;
        case Qt::Key_B:
            ui->balanceCheckbox->setChecked(!ui->balanceCheckbox->isChecked());
            break;
        }
    }

    QMainWindow::keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    // Check if not auto-repeated key release
    if (!event->isAutoRepeat())
    {
        switch (event->key())
        {
        case Qt::Key_W:
            m_forwardPressed = false;
            updateMovement();
            break;
        case Qt::Key_S:
            m_backwardPressed = false;
            updateMovement();
            break;
        case Qt::Key_A:
            m_leftPressed = false;
            updateMovement();
            break;
        case Qt::Key_D:
            m_rightPressed = false;
            updateMovement();
            break;
        }
    }

    QMainWindow::keyReleaseEvent(event);
}

void MainWindow::updateMovement()
{
    if (!m_connection || !m_connection->isConnected())
    {
        return;
    }

    // Get speed factor from slider (0.0 to 1.0)
    double speedFactor = ui->speedSlider->value() / 100.0;

    // Create command based on pressed keys
    QJsonObject cmd;

    if (m_forwardPressed && !m_backwardPressed)
    {
        // Forward movement
        cmd = HexapodProtocol::createBaseCommand("setMovement");
        cmd["direction"] = 0.0; // 0 degrees = forward
        cmd["speed"] = speedFactor;
    }
    else if (m_backwardPressed && !m_forwardPressed)
    {
        // Backward movement
        cmd = HexapodProtocol::createBaseCommand("setMovement");
        cmd["direction"] = 180.0; // 180 degrees = backward
        cmd["speed"] = speedFactor;
    }
    else if (m_leftPressed && !m_rightPressed)
    {
        // Left rotation
        cmd = HexapodProtocol::createBaseCommand("setRotation");
        cmd["direction"] = -1.0; // Counterclockwise
        cmd["speed"] = speedFactor;
    }
    else if (m_rightPressed && !m_leftPressed)
    {
        // Right rotation
        cmd = HexapodProtocol::createBaseCommand("setRotation");
        cmd["direction"] = 1.0; // Clockwise
        cmd["speed"] = speedFactor;
    }
    else if (!m_forwardPressed && !m_backwardPressed && !m_leftPressed && !m_rightPressed)
    {
        // No movement keys pressed, send stop command
        cmd = HexapodProtocol::createBaseCommand("stop");
    }

    // Send the command if we have one
    if (!cmd.isEmpty())
    {
        m_connection->sendCommand(cmd);
    }
}

void MainWindow::on_connectButton_clicked()
{
    if (m_connection->isConnected())
    {
        // Disconnect if already connected
        m_connection->disconnect();
        ui->connectButton->setText("Connect");
        logInfo("Disconnected from server");

        // Stop timers
        m_imuUpdateTimer->stop();
        m_keyProcessTimer->stop();

        // Disable controls
        ui->speedSlider->setEnabled(false);
        ui->pitchSlider->setEnabled(false);
        ui->rollSlider->setEnabled(false);
        ui->forwardButton->setEnabled(false);
        ui->backwardButton->setEnabled(false);
        ui->leftButton->setEnabled(false);
        ui->rightButton->setEnabled(false);
        ui->stopButton->setEnabled(false);
        ui->balanceCheckbox->setEnabled(false);
    }
    else
    {
        // Connect to host
        QString hostname = ui->hostEdit->text();
        int port = ui->portSpin->value();

        logInfo(QString("Connecting to %1:%2...").arg(hostname).arg(port));

        if (m_connection->connectToHost(hostname, port))
        {
            ui->connectButton->setText("Connecting...");
            ui->connectButton->setEnabled(false);

            // Update status indicator
            ui->statusLabel->setText("Connecting...");
            ui->statusLabel->setProperty("status", "connecting");
            ui->statusLabel->update(); // Just update instead of style manipulation
        }
        else
        {
            logError("Failed to initiate connection");
        }
    }
}

void MainWindow::on_actionExit_triggered()
{
    close();
}

void MainWindow::on_actionCalibration_triggered()
{
    QMessageBox::information(this, "Calibration", "Calibration functionality will be implemented in a future update.");
}

void MainWindow::on_actionSettings_triggered()
{
    QMessageBox::information(this, "Settings", "Settings functionality will be implemented in a future update.");
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, "About Hexapod Controller",
                       "Hexapod Remote Control\n"
                       "Version 1.0\n\n"
                       "A Qt-based remote control application for the Hexapod robot.");
}

void MainWindow::on_forwardButton_pressed()
{
    m_forwardPressed = true;
    m_backwardPressed = false;
    updateMovement();
}

void MainWindow::on_backwardButton_pressed()
{
    m_backwardPressed = true;
    m_forwardPressed = false;
    updateMovement();
}

void MainWindow::on_leftButton_pressed()
{
    m_leftPressed = true;
    m_rightPressed = false;
    updateMovement();
}

void MainWindow::on_rightButton_pressed()
{
    m_rightPressed = true;
    m_leftPressed = false;
    updateMovement();
}

void MainWindow::on_stopButton_clicked()
{
    // Reset all movement flags
    m_forwardPressed = false;
    m_backwardPressed = false;
    m_leftPressed = false;
    m_rightPressed = false;

    // Send stop command
    if (m_connection && m_connection->isConnected())
    {
        m_connection->sendCommand(HexapodProtocol::createBaseCommand("stop"));
        logSuccess("Stop command sent");
    }
}

void MainWindow::on_balanceCheckbox_toggled(bool checked)
{
    if (m_connection && m_connection->isConnected())
    {
        QJsonObject cmd = HexapodProtocol::createBaseCommand("setBalanceEnabled");
        cmd["enabled"] = checked;
        m_connection->sendCommand(cmd);
        logInfo(QString("Balance mode %1").arg(checked ? "enabled" : "disabled"));
    }
}

void MainWindow::on_speedSlider_valueChanged(int value)
{
    // Use speedLabel instead of speedValueLabel which doesn't exist
    ui->speedLabel->setText(QString("%1%").arg(value));

    // Apply new speed if movement is active
    updateMovement();
}

void MainWindow::on_pitchSlider_valueChanged(int value)
{
    // Use pitchLabel instead of pitchValueLabel which doesn't exist
    ui->pitchLabel->setText(QString("%1°").arg(value));

    // Apply tilt if connected
    if (m_connection && m_connection->isConnected())
    {
        QJsonObject cmd = HexapodProtocol::createBaseCommand("setTilt");
        cmd["pitch"] = value;
        cmd["roll"] = ui->rollSlider->value();
        m_connection->sendCommand(cmd);
    }
}

void MainWindow::on_rollSlider_valueChanged(int value)
{
    // Use rollLabel instead of rollValueLabel which doesn't exist
    ui->rollLabel->setText(QString("%1°").arg(value));

    // Apply tilt if connected
    if (m_connection && m_connection->isConnected())
    {
        QJsonObject cmd = HexapodProtocol::createBaseCommand("setTilt");
        cmd["pitch"] = ui->pitchSlider->value();
        cmd["roll"] = value;
        m_connection->sendCommand(cmd);
    }
}

void MainWindow::handleConnectionStatus(bool connected)
{
    if (connected)
    {
        logSuccess("Connected to server");
        ui->statusLabel->setText("Connected");
        ui->statusLabel->setProperty("status", "connected");
        ui->connectButton->setText("Disconnect");
        ui->connectButton->setEnabled(true);

        // Enable controls
        ui->speedSlider->setEnabled(true);
        ui->pitchSlider->setEnabled(true);
        ui->rollSlider->setEnabled(true);
        ui->forwardButton->setEnabled(true);
        ui->backwardButton->setEnabled(true);
        ui->leftButton->setEnabled(true);
        ui->rightButton->setEnabled(true);
        ui->stopButton->setEnabled(true);
        ui->balanceCheckbox->setEnabled(true);

        // Start timers
        m_imuUpdateTimer->start();
        m_keyProcessTimer->start();
    }
    else
    {
        ui->statusLabel->setText("Disconnected");
        ui->statusLabel->setProperty("status", "disconnected");
        ui->connectButton->setText("Connect");
        ui->connectButton->setEnabled(true);

        // Stop timers
        m_imuUpdateTimer->stop();
        m_keyProcessTimer->stop();

        // Disable controls
        ui->speedSlider->setEnabled(false);
        ui->pitchSlider->setEnabled(false);
        ui->rollSlider->setEnabled(false);
        ui->forwardButton->setEnabled(false);
        ui->backwardButton->setEnabled(false);
        ui->leftButton->setEnabled(false);
        ui->rightButton->setEnabled(false);
        ui->stopButton->setEnabled(false);
        ui->balanceCheckbox->setEnabled(false);
    }

    // Update status label
    ui->statusLabel->update();
}

void MainWindow::handleResponseReceived(const QJsonObject &response)
{
    // Process response based on type
    if (HexapodProtocol::isSuccessResponse(response))
    {
        // Success response, no need to log all of them
        // But we could extract specific info if needed
        if (response.contains("info"))
        {
            logInfo(response["info"].toString());
        }
    }
    else if (HexapodProtocol::isErrorResponse(response))
    {
        // Error response
        logError(HexapodProtocol::getErrorMessage(response));
    }
    else
    {
        // Other responses
        if (response.contains("message"))
        {
            logInfo(response["message"].toString());
        }
    }
}

void MainWindow::handleImuDataReceived(double accelX, double accelY, double accelZ, double gyroX, double gyroY, double gyroZ)
{
    // Need to create these labels in Qt Designer first before using them
    // For now, comment this code or create labels in the UI file
    /*
    ui->accelXLabel->setText(QString::number(accelX, 'f', 2));
    ui->accelYLabel->setText(QString::number(accelY, 'f', 2));
    ui->accelZLabel->setText(QString::number(accelZ, 'f', 2));
    ui->gyroXLabel->setText(QString::number(gyroX, 'f', 2));
    ui->gyroYLabel->setText(QString::number(gyroY, 'f', 2));
    ui->gyroZLabel->setText(QString::number(gyroZ, 'f', 2));
    */

    // Display IMU data in the status bar instead
    QString imuText = QString("IMU: Accel(%.2f,%.2f,%.2f) Gyro(%.2f,%.2f,%.2f)")
                          .arg(accelX)
                          .arg(accelY)
                          .arg(accelZ)
                          .arg(gyroX)
                          .arg(gyroY)
                          .arg(gyroZ);
    statusBar()->showMessage(imuText, 2000);

    // Either implement updateOrientation in HexapodVisualization class
    // or comment this out until implemented
    // ui->visualizationWidget->updateOrientation(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
}

void MainWindow::handleErrorOccurred(const QString &errorMessage)
{
    // Display error in the log
    logError(errorMessage);
}

void MainWindow::requestImuUpdate()
{
    if (m_connection && m_connection->isConnected())
    {
        m_connection->requestImuData();
    }
}

void MainWindow::logMessage(const QString &message)
{
    QDateTime now = QDateTime::currentDateTime();
    QString timestamp = now.toString("yyyy-MM-dd hh:mm:ss");

    // QTextEdit doesn't have appendPlainText, use append instead
    ui->logTextEdit->append(QString("[%1] %2").arg(timestamp).arg(message));
}

void MainWindow::logInfo(const QString &message)
{
    logMessage("INFO: " + message);
}

void MainWindow::logError(const QString &message)
{
    logMessage("ERROR: " + message);
    // Could also display in status bar or pop-up for critical errors
}

void MainWindow::logSuccess(const QString &message)
{
    logMessage("SUCCESS: " + message);
}
