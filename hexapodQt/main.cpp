#include <QApplication>
#include <QMessageBox>
#include <QSettings>
#include <QFile>
#include <QDir>
#include "mainwindow.h"
#include "hexapodcontext.h"

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

    // Create shared context
    HexapodContext *sharedContext = new HexapodContext(&app);

    // Create and show the main window
    MainWindow mainWindow;
    mainWindow.setContext(sharedContext);
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
    QFile styleFile(":/styles/hexapod.qss");
    if (styleFile.open(QFile::ReadOnly | QFile::Text))
    {
        qApp->setStyleSheet(styleFile.readAll());
        styleFile.close();
    }
}

bool checkRequiredFiles()
{
    // Check for required resources and files
    if (!QFile::exists(":/styles/hexapod.qss"))
    {
        QMessageBox::critical(nullptr, APP_NAME,
                              "Missing required resource files.\n"
                              "Application may not display correctly.");
        // Continue anyway, just with a warning
    }
    return true;
}

void setupApplicationInfo()
{
    // Set application information
    QApplication::setApplicationName(APP_NAME);
    QApplication::setApplicationVersion(APP_VERSION);
    QApplication::setOrganizationName(ORGANIZATION_NAME);
    QApplication::setOrganizationDomain(ORGANIZATION_DOMAIN);
}
