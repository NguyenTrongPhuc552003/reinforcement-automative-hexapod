#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QKeyEvent>
#include <QSettings>
#include <QProgressDialog>
#include "hexapodconnection.h"
#include "hexapodcontext.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // Set shared context
    void setContext(HexapodContext *context);

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void closeEvent(QCloseEvent *event) override;

private slots:
    void on_connectButton_clicked();
    void on_actionExit_triggered();
    void on_actionCalibration_triggered();
    void on_actionSettings_triggered();
    void on_actionAbout_triggered();
    void on_actionTheme_toggled(bool darkMode);
    void on_actionKeyboardShortcuts_triggered();

    void on_forwardButton_pressed();
    void on_backwardButton_pressed();
    void on_leftButton_pressed();
    void on_rightButton_pressed();
    void on_stopButton_clicked();

    void on_balanceCheckbox_toggled(bool checked);
    void on_speedSlider_valueChanged(int value);
    void on_pitchSlider_valueChanged(int value);
    void on_rollSlider_valueChanged(int value);

    void handleConnectionStatus(bool connected);
    void handleResponseReceived(const QJsonObject &response);
    void handleImuDataReceived(double accelX, double accelY, double accelZ,
                               double gyroX, double gyroY, double gyroZ);
    void handleErrorOccurred(const QString &errorMessage);

    void requestImuUpdate();
    void updateMovement();
    void toggleTheme();
    void showKeyboardShortcutsDialog();

    // New connection dialog related slots
    void showConnectionDialog();
    void handleDiscoveredServer(const QString &hostname, int port, bool isSimulation);
    void handleDiscoveryComplete();
    void handleDiscoveryProgress(int current, int total);
    void startServerDiscovery();

private:
    Ui::MainWindow *ui;
    HexapodConnection *m_connection;
    QTimer *m_imuUpdateTimer;
    QTimer *m_keyProcessTimer;
    QSettings m_settings;
    QAction *m_actionTheme;

    bool m_forwardPressed;
    bool m_backwardPressed;
    bool m_leftPressed;
    bool m_rightPressed;
    bool m_darkMode;

    HexapodContext *m_sharedContext;

    void loadSettings();
    void saveSettings();
    void applyTheme();
    void setupKeyboardControls();

    // Helper methods for logging
    void logMessage(const QString &message);
    void logInfo(const QString &message);
    void logError(const QString &message);
    void logSuccess(const QString &message);

    // New UI elements for connection status
    QLabel *m_connectionStatusIndicator;
    QProgressDialog *m_discoveryDialog;
    QStringList m_recentConnections; // List of recently used connections

    void saveRecentConnections();
    void loadRecentConnections();
    void updateConnectionStatusIndicator();
    void addRecentConnection(const QString &hostname, int port);
};

#endif // MAINWINDOW_H
