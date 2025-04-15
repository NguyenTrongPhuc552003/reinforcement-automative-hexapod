#include <QApplication>
#include <QMessageBox>
#include <QSettings>
#include <QFile>
#include <QDir>
#include <QSplashScreen>
#include "mainwindow.h"
#include "hexapodcontext.h"

// Application configuration constants
#define APP_NAME "Hexapod Remote Control"
#define APP_VERSION "1.1.0"
#define ORGANIZATION_NAME "Hexapod Robotics"
#define ORGANIZATION_DOMAIN "hexapodrobotics.com"

// Settings keys
#define SETTINGS_HOSTNAME "connection/hostname"
#define SETTINGS_PORT "connection/port"
#define SETTINGS_WINDOW_GEOMETRY "ui/window_geometry"
#define SETTINGS_WINDOW_STATE "ui/window_state"
#define SETTINGS_BALANCE_MODE "settings/balance_mode"
#define SETTINGS_SPEED_VALUE "settings/speed_value"

void loadStylesheet();
bool checkRequiredFiles();
void setupApplicationInfo();

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // Setup application metadata
    setupApplicationInfo();

    // Create splash screen
    QPixmap splashPixmap(":/images/splash.png");
    if (splashPixmap.isNull())
    {
        splashPixmap = QPixmap(400, 200);
        splashPixmap.fill(QColor("#3498db"));
    }
    QSplashScreen splash(splashPixmap);
    splash.show();
    app.processEvents();

    // Display version info on splash screen
    splash.showMessage(QString("%1 v%2").arg(APP_NAME).arg(APP_VERSION),
                       Qt::AlignBottom | Qt::AlignRight, Qt::white);

    // Load stylesheet if available
    loadStylesheet();

    // Check for required files
    if (!checkRequiredFiles())
    {
        return 1;
    }

    splash.showMessage("Initializing components...", Qt::AlignBottom | Qt::AlignRight, Qt::white);
    app.processEvents();

    // Create shared context
    HexapodContext *sharedContext = new HexapodContext(&app);

    // Load last connection settings
    QSettings settings;
    if (settings.contains(SETTINGS_HOSTNAME))
    {
        sharedContext->setHostname(settings.value(SETTINGS_HOSTNAME).toString());
    }
    if (settings.contains(SETTINGS_PORT))
    {
        bool ok;
        int port = settings.value(SETTINGS_PORT).toInt(&ok);
        if (ok && port > 0)
        {
            sharedContext->setPort(port);
        }
    }

    // Load saved preferences
    if (settings.contains(SETTINGS_BALANCE_MODE))
    {
        sharedContext->setBalanceEnabled(settings.value(SETTINGS_BALANCE_MODE).toBool());
    }
    if (settings.contains(SETTINGS_SPEED_VALUE))
    {
        bool ok;
        double speed = settings.value(SETTINGS_SPEED_VALUE).toDouble(&ok);
        if (ok)
        {
            sharedContext->setSpeed(speed);
        }
    }

    splash.showMessage("Creating user interface...", Qt::AlignBottom | Qt::AlignRight, Qt::white);
    app.processEvents();

    // Create and show the main window
    MainWindow mainWindow;
    mainWindow.setContext(sharedContext);

    // Load window geometry and state from settings
    if (settings.contains(SETTINGS_WINDOW_GEOMETRY))
    {
        mainWindow.restoreGeometry(settings.value(SETTINGS_WINDOW_GEOMETRY).toByteArray());
    }
    if (settings.contains(SETTINGS_WINDOW_STATE))
    {
        mainWindow.restoreState(settings.value(SETTINGS_WINDOW_STATE).toByteArray());
    }

    // Connect aboutToQuit signal to save settings
    QObject::connect(&app, &QApplication::aboutToQuit, [&mainWindow, sharedContext]()
                     {
        QSettings settings;
        settings.setValue(SETTINGS_WINDOW_GEOMETRY, mainWindow.saveGeometry());
        settings.setValue(SETTINGS_WINDOW_STATE, mainWindow.saveState());
        
        // Save connection settings
        settings.setValue(SETTINGS_HOSTNAME, sharedContext->hostname());
        settings.setValue(SETTINGS_PORT, sharedContext->port());
        
        // Save preferences
        settings.setValue(SETTINGS_BALANCE_MODE, sharedContext->isBalanceEnabled());
        settings.setValue(SETTINGS_SPEED_VALUE, sharedContext->speed()); });

    // Finish splash screen and show main window
    splash.finish(&mainWindow);
    mainWindow.show();

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
    else
    {
        // Load a basic stylesheet if the main one is not available
        QFile basicStyle(":/styles/basic.qss");
        if (basicStyle.open(QFile::ReadOnly | QFile::Text))
        {
            qApp->setStyleSheet(basicStyle.readAll());
            basicStyle.close();
        }
    }
}

bool checkRequiredFiles()
{
    // Check for required resources and files
    if (!QFile::exists(":/styles/hexapod.qss"))
    {
        QMessageBox::warning(nullptr, APP_NAME,
                             "Missing stylesheet resource. Using default style.");
        // Continue anyway
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
