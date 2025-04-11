#include <QApplication>
#include <QMessageBox>
#include <QSettings>
#include <QFile>
#include <QDir>
#include "mainwindow.h"

// Application configuration constants
#define APP_NAME "Hexapod Controller"
#define APP_VERSION "1.0.0"
#define ORGANIZATION_NAME "Hexapod Robotics"
#define ORGANIZATION_DOMAIN "hexapodrobotics.com"

// Settings keys
#define SETTINGS_HOSTNAME "connection/hostname"
#define SETTINGS_PORT "connection/port"
#define SETTINGS_WINDOW_GEOMETRY "ui/window_geometry"
#define SETTINGS_WINDOW_STATE "ui/window_state"

void loadStylesheet();
bool checkRequiredFiles();
void setupApplicationInfo();

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // Setup application metadata
    setupApplicationInfo();

    // Load stylesheet if available
    loadStylesheet();

    // Check for required files
    if (!checkRequiredFiles())
    {
        return 1;
    }

    // Create and show the main window
    MainWindow mainWindow;
    mainWindow.show();

    // Load window geometry and state from settings
    QSettings settings;
    if (settings.contains(SETTINGS_WINDOW_GEOMETRY))
    {
        mainWindow.restoreGeometry(settings.value(SETTINGS_WINDOW_GEOMETRY).toByteArray());
    }
    if (settings.contains(SETTINGS_WINDOW_STATE))
    {
        mainWindow.restoreState(settings.value(SETTINGS_WINDOW_STATE).toByteArray());
    }

    // Connect aboutToQuit signal to save settings
    QObject::connect(&app, &QApplication::aboutToQuit, [&mainWindow]()
                     {
        QSettings settings;
        settings.setValue(SETTINGS_WINDOW_GEOMETRY, mainWindow.saveGeometry());
        settings.setValue(SETTINGS_WINDOW_STATE, mainWindow.saveState()); });

    return app.exec();
}

void loadStylesheet()
{
    // Try to load stylesheet if it exists
    QFile file(":/styles/hexapod.qss");
    if (file.exists() && file.open(QFile::ReadOnly | QFile::Text))
    {
        QString style = QLatin1String(file.readAll());
        qApp->setStyleSheet(style);
        file.close();
    }
}

bool checkRequiredFiles()
{
    // Check for required files and directories
    bool allFilesPresent = true;

    // Check if the UI resource file is available in the resource system
    QFile uiFile(":/forms/mainwindow.ui");
    if (!uiFile.exists())
    {
        QMessageBox::warning(nullptr, APP_NAME,
                             "Warning: UI resource file not found. Application may not display correctly.");
        allFilesPresent = false;
    }

    // Create configuration directory if it doesn't exist
    QDir configDir(QDir::homePath() + "/.config/hexapodQt");
    if (!configDir.exists())
    {
        configDir.mkpath(".");
    }

    return allFilesPresent;
}

void setupApplicationInfo()
{
    // Set application metadata
    QCoreApplication::setApplicationName(APP_NAME);
    QCoreApplication::setApplicationVersion(APP_VERSION);
    QCoreApplication::setOrganizationName(ORGANIZATION_NAME);
    QCoreApplication::setOrganizationDomain(ORGANIZATION_DOMAIN);

    // Configure settings format
    QSettings::setDefaultFormat(QSettings::IniFormat);
}
